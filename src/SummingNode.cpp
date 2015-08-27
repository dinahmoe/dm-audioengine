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
 
//  Created by Alessandro Saccoia on 4/13/14.

#include "SummingNode.hpp"

#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include <algorithm>

namespace dinahmoe {
namespace audioengine {

SummingNode::SummingNode(AudioContext* context_, float initialValue1_, float initialValue2_) : AudioNode(context_) {
  addInput("Input 1", TYPE_HYBRID);
  m_inputs[0]->setInitialValue(initialValue1_);
  addInput("Input 2", TYPE_HYBRID);
  m_inputs[1]->setInitialValue(initialValue2_);
  addOutput(1);
}

SummingNode::~SummingNode() {

}

void SummingNode::processInternal(int numSamples, int outputRequesting) {
  const bool isInput1SampleAccurate = m_inputs[0]->isSampleAccurate();
  const bool isInput2SampleAccurate = m_inputs[1]->isSampleAccurate();
  const bool isInput1Silent =  m_inputBuffers[0]->isSilent;
  const bool isInput2Silent = m_inputBuffers[1]->isSilent;
  
  // this is the most likely condition
  m_outputBuffers[0]->isSilent = false;
  
  // isSampleAccurate() has the precedence over isSilent. that is, sometimes
  // isSilent isn't set if isSampleAccurate and the value is zero
  // completely consistent around the framework, sadly. has been introduced
  // a bit later and some nodes are consistent with the model, some not. TODO
  if (isInput1SampleAccurate && isInput2SampleAccurate) {
    if (isInput1Silent && isInput2Silent) {
      m_outputBuffers[0]->isSilent = true;
      return;
    } else if (isInput1Silent) {
      float *pIn = nullptr;
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pIn = m_inputBuffers[1]->data[ch < m_inputBuffers[1]->usedChannels ? ch : 0];
        pOut = m_outputBuffers[0]->data[ch];
        std::copy(pIn, pIn + numSamples, pOut);
      }
      return;
    } else {
      float *pIn = nullptr;
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pIn = m_inputBuffers[1]->data[ch < m_inputBuffers[0]->usedChannels ? ch : 0];
        pOut = m_outputBuffers[0]->data[ch];
        std::copy(pIn, pIn + numSamples, pOut);
      }
      return;
    }
  }
  if (isInput1SampleAccurate) {
    float offset = m_inputs[1]->getValue();
    if (!isInput1Silent) {
      float *pIn = nullptr;
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pOut = m_outputBuffers[0]->data[ch];
        pIn = m_inputBuffers[0]->data[ch < m_inputBuffers[0]->usedChannels ? ch : 0];
        std::transform(pIn, pIn + numSamples, pOut, std::bind2nd(std::plus<float>(), offset));
      }
    } else {
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pOut = m_outputBuffers[0]->data[ch];
        std::fill(pOut, pOut + numSamples, offset);
      }
    }
    return;
  }
  if (isInput2SampleAccurate) {
    float offset = m_inputs[0]->getValue();
    if (!isInput2Silent) {
      float *pIn = nullptr;
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pOut = m_outputBuffers[0]->data[ch];
        pIn = m_inputBuffers[1]->data[ch < m_inputBuffers[0]->usedChannels ? ch : 0];
        std::transform(pIn, pIn + numSamples, pOut, std::bind2nd(std::plus<float>(), offset));
      }
    } else {
      float *pOut = nullptr;
      for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
        pOut = m_outputBuffers[0]->data[ch];
        std::fill(pOut, pOut + numSamples, offset);
      }
    }
    return;
  }
  float offset = m_inputs[0]->getValue() + m_inputs[1]->getValue();
  float *pOut = nullptr;
  for (int ch = 0; ch < m_outputBuffers[0]->usedChannels; ++ch) {
    pOut = m_outputBuffers[0]->data[ch];
    std::fill(pOut, pOut + numSamples, offset);
  }
}

} // audioengine
} // dinahmoe
