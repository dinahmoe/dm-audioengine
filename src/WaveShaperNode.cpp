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
 
//  Created by Alessandro Saccoia on 4/9/14.

#include "WaveShaperNode.hpp"
#include "AudioContext.hpp"
#include <iostream>
#include <cstring>

namespace dinahmoe {
namespace audioengine {

WaveShaperNode::WaveShaperNode(AudioContext* context) :
  AudioNode(context) {
	addInput("Audio input", TYPE_AUDIO);
	addOutput(2);
  m_coeff = DM_WAVESHAPER_CURVE_SIZE / 2;
  float inverse = 2.F / (float)DM_WAVESHAPER_CURVE_SIZE;

  for (int i = 0; i < DM_WAVESHAPER_CURVE_SIZE; ++i) {
    m_curve[i] = ((float)i * inverse) - 1.F;
  }
}

WaveShaperNode::~WaveShaperNode() {

}

void WaveShaperNode::processInternal(int numSamples, int outputRequesting) {
  if (m_inputBuffers[0]->isSilent) {
    m_outputBuffers[0]->isSilent = true;
    return;
  }
   m_outputBuffers[0]->isSilent = false;
  for (size_t chan = 0; chan < m_outputBuffers[0]->usedChannels; ++chan) {
    float* pIn = m_inputBuffers[0]->data[chan];
    float* pOut = m_outputBuffers[0]->data[chan];
    int index;
    for (int i = 0; i < numSamples; ++i) {
      index =  m_coeff * (*pIn + 1);
      index = std::max(index, 0);
      index = std::min(index, DM_WAVESHAPER_CURVE_SIZE - 1);
      *pOut = m_curve[index];
      ++pIn, ++pOut;
    }
  }
}

void WaveShaperNode::setCurve(float* buffer) {
  memcpy(m_curve, buffer, DM_WAVESHAPER_CURVE_SIZE * sizeof(float));
}


}
}
