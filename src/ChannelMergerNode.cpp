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

#include "ChannelMergerNode.hpp"
#include "AudioContext.hpp"
#include "AudioNodeInput.hpp"

namespace dinahmoe {
namespace audioengine {

ChannelMergerNode::ChannelMergerNode(AudioContext* context, size_t inputChannels_) :
  AudioNode(context),
  m_inputChannels(inputChannels_) {
  assert(inputChannels_ > 0);
  char buf[8];
  for (size_t input = 0; input < m_inputChannels; ++input) {
    sprintf((char*)buf, "%zi", input);
    addInput(buf, TYPE_AUDIO);
  }
  addOutput(inputChannels_);
}

ChannelMergerNode::~ChannelMergerNode() {

}

// again a NOP: when we attach with a  source to one of the inputs,
// we should avoid the default behaviour of having the used output channels
// to be assigned to 1: the need to remain in the starting configuration
void ChannelMergerNode::process_inputChannelsChanged(unsigned int inputNumber_) {

}

void ChannelMergerNode::processInternal(int numSamples, int outputRequesting) {
  bool isSilent = true;
  for (unsigned int inCh = 0; inCh < m_inputChannels; ++inCh) {
    if (!m_inputBuffers[inCh]->isSilent) {
      isSilent = false;
      break;
    }
  }
  if (isSilent) {
    m_outputBuffers[0]->isSilent = true;
    return;
  }
  AudioBuffer* out = m_outputBuffers[0];
  out->isSilent = false;
  for (unsigned int inCh = 0; inCh < m_inputChannels; ++inCh) {
    const AudioBuffer* in = m_inputBuffers[inCh];
    float* src;
    if (in->isSilent) {
      src = m_context->getSilentBuffer();
    } else {
      src = in->data[0];
    }
    std::copy(src, src + numSamples, out->getChannelData(inCh));
  }
}

}
}
