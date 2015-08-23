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

#ifndef AudioEngine_AudioBufferSourceNode_hpp
#define AudioEngine_AudioBufferSourceNode_hpp

#include <mutex>
#include "AudioNode.hpp"

namespace dinahmoe {
namespace audioengine {

class InMemoryBuffer;

class AudioBufferSourceNode :
	public AudioNode {
public:	
	AudioBufferSourceNode(AudioContext* context, float playbackRate_);
	~AudioBufferSourceNode();
  inline const char* getType() { return "AudioBufferSourceNode"; }

	void processInternal(int numSamples, int outputRequesting);
  
  enum PlaybackState { UNSCHEDULED, SCHEDULED, PLAYING, STOPPED, FINISHED };
  enum LoopMode { NOLOOP, LOOP_BUFFER }; // improve to taste
  
  void noteOn(dm_time_seconds time, float offset = 0.F, float duration = .0F);
  
  void noteOff(dm_time_seconds time);
  
  void setBuffer(InMemoryBuffer* buffer);
  
  InMemoryBuffer* const getBuffer() const;
  
  void setLoopMode(LoopMode mode) {
    m_loopMode = mode;
  }
  
  // called by the buffer on this
  void bufferReady(InMemoryBuffer* buffer);
  
  char* name();
  
  PlaybackState& playbackState() { return m_state; }
  
  dm_time_seconds currentPosition();
  
private:  
  InMemoryBuffer* m_buffer;
  LoopMode m_loopMode;
  PlaybackState m_state;
  unsigned long m_start;
  unsigned long m_end;
  unsigned long m_bufferEnd;
  // this has the same units as dm_time_seconds but it's fractional
  double m_positionInBuffer;
  std::mutex m_mutex;
};

} // AUDIOENGINE
} // DMAF

#endif