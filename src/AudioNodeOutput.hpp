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
 
/*
 * Written by Alessandro Saccoia 22/08/2012
 * alessandro@dinahmoe.com
 */

#ifndef __AUDIONODEOUTPUT_HPP__
#define __AUDIONODEOUTPUT_HPP__

#include <list>
#include "AudioBuffer.hpp"

namespace dinahmoe {
namespace audioengine {

class AudioNode;
class AudioNodeInput;

class AudioNodeOutput {
public:
	AudioNodeOutput(AudioNode* node, int outputIndex, int channels);
	
	~AudioNodeOutput();
	
  AudioNode* node() {
    return m_node;
  }
  
	void addInput(AudioNodeInput* input);
	
	void disconnect();
	
	// this gets called by the inputs of the nodes we are connected to
	// trigger a process call if this output hasn't processed in this render
	// quantum
	AudioBuffer* pull(size_t numFrames);
	
  void updateInternalState();
  
  // this is the method that shall be called by the audionodes like convolver
  // to set their channel count when something changed. the changes will be
  // picked up at the next process call so it's preferable to give out an empty
  void setChannels(int numberOfChannels);
  
  inline int getChannels() { return m_channels; }
  
private:
	AudioNode* m_node;
	int m_channels;
	int m_outputIndex;
  
  int m_futureChannels;

	std::list<AudioNodeInput*> m_inputs;
	
	// this vector reflects the most current list of outputs
	// and it's modified just from the realtime thread
	// during the call to updateInternalState
	std::list<AudioNodeInput*> m_destinationInputs;

};

} // AudioEngine
} // DMAF

#endif
