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
 
//  Created by Alessandro Saccoia on 2/25/14.

#include "ChannelSplitterNode.hpp"
#include "AudioContext.hpp"
#include "AudioNodeInput.hpp"

namespace dinahmoe {
namespace audioengine {

ChannelSplitterNode::ChannelSplitterNode(AudioContext* context, size_t outputChannels_) :
  AudioNode(context),
  m_outputChannels(outputChannels_) {
	addInput("Audio input", TYPE_AUDIO);
  for (size_t output = 0; output < m_outputChannels; ++output) {
    addOutput(1);
  }
}

ChannelSplitterNode::~ChannelSplitterNode() {

}

void ChannelSplitterNode::processInternal(int numSamples, int outputRequesting) {
  const AudioBuffer* in = m_inputBuffers[0];
  if (in->isSilent) {
    m_outputBuffers[outputRequesting]->isSilent = true;
    return;
  }
  float* src;
  if (outputRequesting < (int)in->usedChannels) {
    src = in->getChannelData(outputRequesting);
  } else {
    src = m_context->getSilentBuffer();
  }
  std::copy(src, src + numSamples, m_outputBuffers[outputRequesting]->data[0]);
  m_outputBuffers[outputRequesting]->isSilent = false;
}

}
}
