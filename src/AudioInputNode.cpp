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
 
//  Created by Alessandro Saccoia on 7/29/13.

#include "AudioInputNode.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeInputType.hpp"
#include <functional>
#include <algorithm>

/*

  Audio Output      
   ___0___
  |       |
  |_______|
    0    1
    
    0: audio input
    1: control input (HYBRID)
 
*/

namespace dinahmoe {
namespace audioengine {

AudioInputNode::AudioInputNode(AudioContext* context) :
  AudioNode(context),
  m_internalBuffer(context->getOutputChannels(), this->context()->getBufferSize()){
	addOutput(1);
//! [Adding inputs]
}

AudioInputNode::~AudioInputNode() {
	
}

void AudioInputNode::setBuffers(float** buf, int channels) {
  for (int chan = 0; chan < channels; ++chan) {
    std::copy(buf[chan], buf[chan] + m_internalBuffer.size, m_internalBuffer.data[chan]);
  }
  m_internalBuffer.usedChannels = channels;
  m_internalBuffer.isSilent = false;
}

void AudioInputNode::processInternal(int numSamples, int outputRequesting) {
   *(m_outputBuffers[0]) = m_internalBuffer;
}

} //AudioEngine
} // DMAF
