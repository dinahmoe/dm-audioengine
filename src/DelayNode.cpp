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
 
//  Created by Alessandro Saccoia on 6/24/13.

#include "DelayNode.hpp"

#include <cstring>

#include "AudioContext.hpp"
#include "AudioNodeInput.hpp"

namespace dinahmoe {
namespace audioengine {

DelayNode::DelayNode(AudioContext* context, float maxDelay, float initialDelay) :
  AudioNode(context),
  m_bufferWriteIndex(0),
  m_samplingRate(context->getSampleRate()) {
	addInput("Audio input", TYPE_AUDIO);
  addInput("Delay control", TYPE_HYBRID);
  m_inputs[1]->setInitialValue(initialDelay);
  addOutput(1);
  m_maxDelay_s = static_cast<int>(ceil(maxDelay * context->getSampleRate()));
  for (int i = 0; i < 2; ++i) {
    m_delayLine[i] = new float[m_maxDelay_s];
  }
  clear();
}

void DelayNode::clear() {
  for (int i = 0; i < 2; ++i) {
    memset((void*)(m_delayLine[i]), 0, sizeof(float) * m_maxDelay_s);
  }
}

DelayNode::~DelayNode() {
  for (int i = 0; i < 2; ++i) {
    delete [] m_delayLine[i];
  }
}

//http://music.columbia.edu/pipermail/music-dsp/2012-April/070656.html
// todo make powers of two and avoid all the branching
// cache the calls to getChannelData
void DelayNode::processInternal(int numSamples, int outputRequesting) {
  int currentChannels = m_outputBuffers[0]->usedChannels;
  size_t previous, next;
  float interpRatio;
  float currentDelay_s;
  float temp;
  const AudioBuffer* in = m_inputBuffers[0];
  AudioBuffer* out = m_outputBuffers[0];
  out->isSilent = false;
  bool sampleAccurate = m_inputs[1]->isSampleAccurate();
  float *timingData;
  if (sampleAccurate) {
   timingData = m_inputBuffers[1]->data[0];
  } else {
   currentDelay_s = m_inputs[1]->getValue() * m_samplingRate;
   if (currentDelay_s < 0 || currentDelay_s > m_maxDelay_s) {
     currentDelay_s = 0;
     dmaf_log(m_context->getLog(), Log::Warning, "Delay value out of bounds");
   }
  }

  for (int sample = 0; sample < numSamples; ++sample) {
    if (sampleAccurate) {
      currentDelay_s = *(timingData++) * m_samplingRate;
      if (currentDelay_s < 0 || currentDelay_s > m_maxDelay_s) {
        currentDelay_s = 0;
        dmaf_log(m_context->getLog(), Log::Warning, "Delay value out of bounds");
      }
    }
    temp = (float)m_bufferWriteIndex - currentDelay_s;
    if (temp < 0.0F) {
      temp = (float)m_maxDelay_s + temp;
    }
    previous = (size_t)(floor(temp));
    next = (previous + 1) % m_maxDelay_s;
    interpRatio = temp - (float)previous;
    for (int ch = 0; ch < currentChannels; ++ch) {
      if (in->isSilent)
        m_delayLine[ch][m_bufferWriteIndex] = .0F;
      else
        m_delayLine[ch][m_bufferWriteIndex] = in->data[ch][sample];
      out->data[ch][sample] = ((1.0F - interpRatio) * m_delayLine[ch][previous]) + (interpRatio * m_delayLine[ch][next]);
    }
    ++m_bufferWriteIndex;
    m_bufferWriteIndex %= m_maxDelay_s;
  }
}

}
}
