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
 
#include "AudioContext.hpp"
#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeOutput.hpp"
#include "ControlTimeline.hpp"
#include "Log.h"

namespace dinahmoe {
namespace audioengine {

AudioNodeInput::AudioNodeInput(AudioNode* node,
  const char* name_,
  const AudioNodeInputType& type_,
  const int inputIndex,
  const float defaultValue) :
  m_node(node),
  m_timeline(new ControlTimeline(this, defaultValue)),
  m_name(name_),
  m_type(type_),
  m_inputIndex(inputIndex),
  m_currentChannels(0),
  m_renderingStateNeedUpdating(false),
  m_internalBuffer(node->context()->getOutputChannels(), node->context()->getBufferSize()) {
  
  if (m_type == TYPE_AUDIO) {
    m_currentType = TYPE_AUDIO;
    m_isSampleAccurate = true;
  } else {
    m_currentType = TYPE_CONTROL;
    m_isSampleAccurate = false;    
  }
}

AudioNodeInput::~AudioNodeInput() {
  
}

// this is thread safe, lock is called during the audionode connect() function
void AudioNodeInput::connect(AudioNodeOutput* source) {
  assert(m_type != TYPE_CONTROL);
  // source already connected: return
  if (std::find(m_sources.begin(), m_sources.end(), source) != m_sources.end()) {
//    dm_debug_log(node()->context()->getLog(), Log::Error, "Connecting an ANOutput already connected or waiting to be disconnected");
    return;
  }
  // connect called twice before the process gets things sorted: return
  if (std::find(m_sourcesDirty.begin(), m_sourcesDirty.end(), source) != m_sourcesDirty.end()) {
//    dm_debug_log(node()->context()->getLog(), Log::Error, "Connecting an ANOutput already connected");
    return;
  }
	
  m_sourcesDirty.push_back(source);
	
  node()->incrementRefCount(AudioNode::RefTypeConnection);
  
  m_renderingStateNeedUpdating = true;  

  // signal the audio thread that something has changed
  m_node->context()->markForUpdate(this);
}

void AudioNodeInput::disconnect(AudioNodeOutput* output) {
  //dmaf_log(Log::Error, "%p::disconnect(%p)", this, output);
  assert(m_type != TYPE_CONTROL);
  if (std::find(m_sourcesDirty.begin(), m_sourcesDirty.end(), output) == m_sourcesDirty.end()) {
    dm_debug_log(node()->context()->getLog(), Log::Error, "Can't find the ANOutput in the dirty list of ANOutputs for this node. something didn't work properly");
    return;
  }
  m_sourcesDirty.erase(std::find(m_sourcesDirty.begin(), m_sourcesDirty.end(), output));
  node()->decrementRefCount(AudioNode::RefTypeConnection);
  m_renderingStateNeedUpdating = true;
  m_node->context()->markForUpdate(this);
}

void AudioNodeInput::updateChannelCount() {
  assert(node()->context()->canUpdateGraph());
  int newChannelCount = getMaxChannels();
  if (m_currentChannels != newChannelCount) {
    m_currentChannels = newChannelCount;
    node()->inputChannels[m_inputIndex] = newChannelCount;
    if (m_node->m_state != AudioNode::DISAPPEARING) {
      node()->process_inputChannelsChanged(m_inputIndex);
    }
    m_internalBuffer.setUsedChannels(m_currentChannels);
  }
}
	
void AudioNodeInput::updateInternalState() {
  if (m_node->m_state == AudioNode::DISAPPEARING) {
    node()->context()->getAudioNodeOutputsPool().returnToPool(m_sources, m_sources.begin(), m_sources.end());
    return;
  }
  assert(node()->context()->canUpdateGraph());
  if (m_renderingStateNeedUpdating) {
    // this could be optimized but for now it's at least relatively clear
    node()->context()->getAudioNodeOutputsPool().returnToPool(m_sources, m_sources.begin(), m_sources.end());
    if (m_sourcesDirty.size()) {
      auto its = node()->context()->getAudioNodeOutputsPool().pushBackInList(m_sources, m_sourcesDirty.size());
      for (auto src: m_sourcesDirty) {
        *its = src;
        ++its;
      }
    }
    if (m_sources.size() > 0) m_node->m_state = AudioNode::ACTIVE;
    updateChannelCount();    
    if (m_type == TYPE_HYBRID) {
      if (m_sources.size() > 0) {
        m_currentType = TYPE_AUDIO;
      } else {
        m_currentType = TYPE_CONTROL;
      }
    }    
    m_renderingStateNeedUpdating = false;
  }
}

int AudioNodeInput::getMaxChannels() {
  int maxChan = 0;
  for (auto src: m_sources) {
    maxChan = std::max(src->getChannels(), maxChan);
  }
  return maxChan;
}
// if the type is audio, try to pull from the sources or return a silent buffer
// if the type is control:
// - if it's sample accurate fill the internal buffer and set m_isSampleAccurate to true
// - if it is not, set m_isSampleAccurate to false
AudioBuffer const * AudioNodeInput::pull(size_t numFrames)
{
  if (m_currentType == TYPE_AUDIO) {    
    // No connections: return the address of the default silent address
    if (!m_sources.size()) {
      m_internalBuffer.isSilent = true;;
      return &m_internalBuffer;        
    }
    // Single connection: return the address of the buffer of the 
    // only output connected to us
    int nSources = m_sources.size();
    auto currentSrc = m_sources.begin();
    AudioBuffer* output = (*currentSrc++)->pull(numFrames);
    if (nSources == 1) {
      return output;
    }
    // Multiple connections: return the address of the internal buffer
    // we can't use the other node's one because it contains data that can
    // be used by other inputNodes
    // ATTN: copy operator as defined in AudioBuffer!
    //m_internalBuffer = *output;
    // LONGER SLOWER VERSION
//    m_internalBuffer.zero(); //zero it before
//    m_internalBuffer.sum(*output, numFrames);
    m_internalBuffer.zeroAndcopy(*output);

    AudioBuffer* srcBuffer;
    while (currentSrc != m_sources.end()) {
      srcBuffer =(*currentSrc++)->pull(numFrames);
      if (srcBuffer->isSilent)
        continue;
      m_internalBuffer.sum(*srcBuffer, numFrames);
    }    
    return &m_internalBuffer;
  } else if (m_currentType == TYPE_CONTROL) {
    if (m_timeline->hasEvents(numFrames)) {
      // TODO: awesome, this is a race condition: the main thread can (and has) canceled the
      // values in the event set between the hasEvents call and this call. This is because the
      // main thread should never access directly the timeline, as it's doing now!
      // so, create a new event for the cancelValue event and enqueue it in the new event set!
      m_timeline->pull(m_internalBuffer, numFrames);
      m_isSampleAccurate = true;
      return &m_internalBuffer;    
    }
    // this returns NULL because the node shouldn't access
    // this as it contains garbage. This node is CONTROL and
    // has no timeline values, so the node after checking 
    // isSampleAccurate should call getValue
    m_isSampleAccurate = false;
    return NULL;
  }
  assert(false);
  return NULL;
}

const float AudioNodeInput::getValue() const {
  assert(m_currentType == TYPE_CONTROL);
  return m_timeline->getLastValue();
}

void AudioNodeInput::setValueImmediate(float value_, dm_time_seconds time_) {
  assert(m_currentType == TYPE_CONTROL);
  m_timeline->setValueAtTime(value_, time_);
}

void AudioNodeInput::linearRampToValueAtTime(float value_, dm_time_seconds time_) {
  assert(m_currentType == TYPE_CONTROL);
  m_timeline->linearRampToValueAtTime(value_, time_);
}

void AudioNodeInput::cancelScheduledValues(dm_time_seconds time_) {
  assert(m_currentType == TYPE_CONTROL);
  m_timeline->cancelScheduledValues(time_);
}

void AudioNodeInput::setInitialValue(float value_) {
  assert(m_currentType == TYPE_CONTROL);
  m_timeline->m_lastValue = value_;
}
 
} //AudioEngine
} // DMAF
