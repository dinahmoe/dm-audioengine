/*
 * Copyright (c) 2015, Dinahmoe. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
//  Created by Alessandro Saccoia on 8/28/12.

#include "AudioBufferSourceNode.hpp"

#include <functional>
#include <algorithm>
#include <iostream>
#include <cassert>

#include "InMemoryBuffer.hpp"
#include "AudioContext.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeOutput.hpp"
#include "Log.h"


namespace dinahmoe {
namespace audioengine {

AudioBufferSourceNode::AudioBufferSourceNode(AudioContext* context, float playbackRate_) :
  AudioNode(context),
  m_buffer(NULL),
  m_loopMode(NOLOOP),
  m_start(0),
  m_end(std::numeric_limits<unsigned long>::max()),
  m_positionInBuffer(.0F) {
  addInput("Playback Rate", TYPE_HYBRID);
  m_inputs[0]->setInitialValue(playbackRate_);
	addOutput(1);

}

AudioBufferSourceNode::~AudioBufferSourceNode() {

}


void AudioBufferSourceNode::processInternal(int numSamples, int outputRequesting) {
  if (m_mutex.try_lock()) {
    if (!m_buffer || m_state == UNSCHEDULED || m_state == FINISHED) {
      m_outputBuffers[0]->isSilent = true;
      m_mutex.unlock();
      return;
    }

    // to be sure to have a consistent state of the variables across this routine
    // take a copy in the beginning. there are no dependencies between them so
    // this is ok to do without mutexes, the worst thing that can happen is that
    // one of the two will be picked up at the next render cycle

    LoopMode loopMode = m_loopMode;
    unsigned long endFrame = std::min(m_end, m_bufferEnd);
    unsigned long cachedEnd = m_end;
    unsigned long thisQuantum = context()->getCurrentSampleFrames();
    unsigned long nextQuantum = thisQuantum + numSamples;
    // toBeStreamed is always the number of real audio samples that we need to output
    // no matter what the playback rate is
    size_t toBeStreamed = numSamples;
    int startInserting = 0;
    if (m_state == SCHEDULED) {
      if (m_start < thisQuantum) {
        m_start = thisQuantum;
        startInserting = 0;
        toBeStreamed = nextQuantum - m_start;
        m_state = PLAYING;
        ++m_context->m_currentPoliphony;
      } else if (m_start < nextQuantum && m_start >= thisQuantum) {
        // in this case we get the beginning perfectly
        startInserting = m_start - thisQuantum; // fill till we need to start inserting samples
        m_outputBuffers[0]->zero(startInserting);
        toBeStreamed = nextQuantum - m_start;
        m_state = PLAYING;
        ++m_context->m_currentPoliphony;
      } else if (m_start < nextQuantum) {
        // we are late, start immediately TODO: review
        dm_debug_log(context()->getLog(), Log::Debug, "Playing late and it will cut the tail of the sound. The scheduler wanst keeping up.");
        //m_bufferEnd += nextQuantum - m_start;
        m_state = PLAYING;
      } else {
        m_outputBuffers[0]->isSilent = true;
        m_mutex.unlock();
        return;
      }
    }
    bool isEnding = false;
    size_t zeroedSamples = 0;
    // we are at this line just we are playing
    if (loopMode == NOLOOP && endFrame < nextQuantum) {
      zeroedSamples = std::min(nextQuantum - endFrame - 1, (unsigned long)numSamples);
      isEnding = true;
    } else if (loopMode == LOOP_BUFFER && cachedEnd < nextQuantum) {
      zeroedSamples = std::min(nextQuantum - cachedEnd - 1, (unsigned long)numSamples);
      isEnding = true;
    }

    if (isEnding) {
      if (zeroedSamples > (size_t)numSamples) {
        // we have probably glitched! TODO find out exactly what happens!
        dm_debug_log(context()->getLog(), Log::Error, "SoundBuffers: If we are here the audio engine has skipped a frame. Finishing sample.");
        m_state = FINISHED;
        m_outputBuffers[0]->isSilent = true;
        context()->sourceNodeCompleted(this);
        --m_context->m_currentPoliphony;
        m_mutex.unlock();
        return;
      }
      if (zeroedSamples > 0) { // avoid a memset with 0 samples
        // zero the last samples right now
        m_outputBuffers[0]->zero(zeroedSamples, numSamples - zeroedSamples);
        // ramaining to be streamed
        toBeStreamed -= zeroedSamples;
      }
      m_state = FINISHED;
    }

    if (m_inputs[0]->isSampleAccurate()) {
      // TODO Frequency Modulation
      assert(false);
    }
    //dmaf_log(Log::Error, "Position %p %f %i ", this, m_positionInBuffer, thisQuantum);
    // TODO the playback speed changes the m_end!
    double playBackSpeed = m_inputs[0]->getValue();

    unsigned long wavetable_index = (unsigned long)floor(m_positionInBuffer);
    double fractional_index = m_positionInBuffer - (double)wavetable_index;
    
    float* outChannelPointers[2];
    float* inChannelPointers[2];
    for (size_t j = 0; j < m_buffer->buffer.channels; ++j) {
      outChannelPointers[j] = (m_outputBuffers[0]->data[j] + startInserting);
      inChannelPointers[j] = m_buffer->buffer[j];
    }
    
    for (size_t i = 0; i < toBeStreamed; ++i) {
      if (playBackSpeed == 1.0F) {
        for (size_t j = 0; j < m_buffer->buffer.channels; ++j) {
          *((*(outChannelPointers + j))++) = *(inChannelPointers[j] + (int)m_positionInBuffer);
        }
        m_positionInBuffer += playBackSpeed;
      } else {
        // linear interpolation
        // when you will wanna do real wavetables with size 2^N take a look at
        // R-B-J http://music.columbia.edu/pipermail/music-dsp/2012-April/070656.html
        for (size_t j = 0; j < m_buffer->buffer.channels; ++j) {
//          *((*(outChannelPointers + j))++) = m_buffer->buffer[j][wavetable_index] +
//            fractional_index * (m_buffer->buffer[j][wavetable_index + 1] - m_buffer->buffer[j][wavetable_index]);
//          
          *((*(outChannelPointers + j))++) = m_buffer->buffer[j][wavetable_index] +
            fractional_index * (m_buffer->buffer[j][wavetable_index + 1] - m_buffer->buffer[j][wavetable_index]);
        }
        m_positionInBuffer += playBackSpeed;
        wavetable_index = (unsigned long)floor(m_positionInBuffer);
        fractional_index = m_positionInBuffer - (float)wavetable_index;
        // don't check the bounds if we are on the last sample, because that memory won't be accessed!
        assert ((wavetable_index < m_buffer->buffer.size) || (i == toBeStreamed-1));
      }

      if (m_positionInBuffer  > m_buffer->buffer.size - 1) { // TODO OPTIMIZE this branch in the inner loop
        // TODO shall I floor this... the rounding to size_t is probably platform dependent,
        // and
        m_positionInBuffer = fmod(m_positionInBuffer, (float)m_buffer->buffer.size);
      }
    }

    m_outputBuffers[0]->isSilent = false;

    if (m_state == FINISHED) {
      // ALEX TODO: race condition
      // we don't have the lock to protect the activeSourceNodes...
      // hopefully nobody will mess up the vector in the same moment
      // but this has to be done on the main thread!
      // YES... we can't mess up with input outputs during a process call...
      //m_outputs[0]->disconnect();
      context()->sourceNodeCompleted(this);
      --m_context->m_currentPoliphony;
    }
    m_mutex.unlock();
  } else {
    m_outputBuffers[0]->zero(numSamples);
  }
}

void AudioBufferSourceNode::setBuffer(InMemoryBuffer* buffer) {
  m_mutex.lock();
  m_buffer = buffer;
  if (!m_buffer->isReady()) {
    m_state = UNSCHEDULED;
  } else {
    // all the loading in synchronous
    // so if the buffer is good we always get here.
    // we are in the main thread, we can't mess up with the node inputs and configurations.
    // if a processing call arrives right now, we have a problem.
    this->m_outputs[0]->setChannels(m_buffer->buffer.channels);
    m_state = UNSCHEDULED;
  }
  m_positionInBuffer = 0.0;
  m_mutex.unlock();

  // the process call can enter and the first thing will be
  // to cleanup all the channel counts before pulling this node, so OK!
}

InMemoryBuffer* const AudioBufferSourceNode::getBuffer() const {
  return m_buffer;
}

void AudioBufferSourceNode::bufferReady(InMemoryBuffer* buffer) {
  if (buffer == m_buffer) {
    // TODO for async loading
  }
}


void AudioBufferSourceNode::noteOn(dm_time_seconds time, float offset, float duration) {
  if (m_buffer == NULL) {
    dm_debug_log(context()->getLog(), Log::Warning, "Tried to schedule a note whose buffer is not assigned yet");
    return;
  }
  m_start = static_cast<unsigned long>(round(time * context()->getSampleRate()));
  assert(offset < m_buffer->duration());
  double playBackSpeed = m_inputs[0]->getValue();
  if (duration == .0F) {
    m_bufferEnd =  static_cast<unsigned long>(m_start + (floor(m_buffer->buffer.size / playBackSpeed))) - 1;
  } else {
    size_t duration_s = static_cast<unsigned long>(round(offset * context()->getSampleRate()));
    m_bufferEnd =  static_cast<unsigned long>(m_start + (floor(duration_s / playBackSpeed))) - 1;
  }
  m_positionInBuffer = static_cast<unsigned long>(round(offset * context()->getSampleRate()));
  m_state = SCHEDULED;
}

void AudioBufferSourceNode::noteOff(dm_time_seconds time) {
  float newEnd = static_cast<unsigned long>(round(time * context()->getSampleRate()));
  if (!(m_loopMode == NOLOOP && newEnd >= m_bufferEnd)) {
    m_end = newEnd;
  }
}

dm_time_seconds AudioBufferSourceNode::currentPosition() {
  assert (m_buffer != NULL);
  return m_positionInBuffer / m_buffer->sampleRate;
}



} //AudioEngine
} // DMAF
