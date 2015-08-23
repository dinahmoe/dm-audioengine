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
 

#ifndef __AUDIONODEINPUT_HPP__
#define __AUDIONODEINPUT_HPP__

#include <list>
#include <string>
#include "AudioBuffer.hpp"
#include "AudioNodeInputType.hpp"
#include "AudioNode.hpp"
#include "AudioContext.hpp"

namespace dinahmoe {
namespace audioengine {

class AudioNodeOutput;
class ControlTimeline;

class AudioNodeInput {
public:
#pragma mark User code interface
  void setValueImmediate(float value_, dm_time_seconds time_);
  void linearRampToValueAtTime(float value_, dm_time_seconds time_);
  void cancelScheduledValues(dm_time_seconds time_);
  void setInitialValue(float value_);
  
#pragma mark Internal interface handled mostly by AudioNode

	AudioNodeInput(AudioNode* node, 
    const char* name_,
    const AudioNodeInputType& type_, 
    const int inputIndex,
    const float defaultValue = 1.F);
    
	~AudioNodeInput();
	
  AudioNode* node() const {
    return m_node;
  }
  void connect(AudioNodeOutput* output);  
  void disconnect(AudioNodeOutput* output);
	void updateInternalState();
  void updateChannelCount();
  
  // this is const because the algorithm should never change
  // the pulled data as it could be used by other nodes
	AudioBuffer const * pull(size_t numFrames);
  
// properties
  
  const char* getName() const {
    return m_name;
  }
  const AudioNodeInputType getType() const {
    return m_type;
  }
  const AudioNodeInputType getCurrentType() const {
    return m_currentType;
  }
  const float getValue() const;
  
  // returns wether or not there are sample accurate values for this frame
  const bool isSampleAccurate() const {
    assert(node()->context()->isAudioThread());
    return (m_currentType == TYPE_AUDIO || (m_currentType == TYPE_CONTROL && m_isSampleAccurate));
  }
  
private:
  
  // computes the max number of channels of the source nodes
  // and stores it in the node()'s 
  int getMaxChannels();
	AudioNode* m_node;
  
protected:
  std::unique_ptr<ControlTimeline> m_timeline;
private:
  const char* m_name;
  const AudioNodeInputType m_type;
  AudioNodeInputType m_currentType;
  bool m_isSampleAccurate;
  
  // m_inputIndex is used to signal the node that someone has
  // connected to us and maybe trigger a reconfiguration
  // of the channels
  int m_inputIndex;
  int m_currentChannels;
  bool m_renderingStateNeedUpdating;
    
	std::list<AudioNodeOutput*> m_sources;
	
	// this vector reflects the most current list of outputs
	// and it's modified just from the realtime thread
	// during the call to updateInternalState
	std::list<AudioNodeOutput*> m_sourcesDirty;
public:
	AudioBuffer m_internalBuffer;  

};

} // AUDIOENGINE
} // DMAF

#endif