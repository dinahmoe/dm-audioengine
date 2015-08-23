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
 
#include "AudioGainNode.hpp"
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

AudioGainNode::AudioGainNode(AudioContext* context, float gain_) :
  AudioNode(context) {
//! [Adding inputs]
	addInput("Audio input", TYPE_AUDIO);
  addInput("Gain control", TYPE_HYBRID);
  m_inputs[1]->setInitialValue(gain_);
	addOutput(1);
//! [Adding inputs]
}

AudioGainNode::~AudioGainNode() {

}

void AudioGainNode::processInternal(int numSamples, int outputRequesting) {
  if (m_inputBuffers[0]->isSilent || !(m_outputBuffers[0]->usedChannels)) {
    m_outputBuffers[0]->isSilent = true;
    return;
  }
	for(size_t nChannel = 0; nChannel < m_outputBuffers[0]->usedChannels; ++nChannel) {
		float *pIn = m_inputBuffers[0]->getChannelData(nChannel);
		float *pOut = m_outputBuffers[0]->getChannelData(nChannel);
    if (m_inputs[1]->isSampleAccurate()) {
      // NB: getChannelData itself knows if there is a timeline or not
      float *pGain = m_inputBuffers[1]->data[0];
      std::transform(pIn, pIn + numSamples, pGain, pOut, std::multiplies<float>());
      m_outputBuffers[0]->isSilent = false;
    } else {
      float gain = m_inputs[1]->getValue();
      if (gain == 0.F) {
        m_outputBuffers[0]->isSilent = true;
        return;
      } else if (gain == 1.F) {
        std::copy(pIn, pIn + numSamples, pOut);
        m_outputBuffers[0]->isSilent = false;
      } else {
        std::transform(pIn, pIn + numSamples, pOut, std::bind2nd(std::multiplies<float>(), gain));
        m_outputBuffers[0]->isSilent = false;
      }
    }

	}
}

} //AudioEngine
} // DMAF
