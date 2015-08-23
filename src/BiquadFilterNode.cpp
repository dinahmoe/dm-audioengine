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
 
//  Created by Alessandro Saccoia on 6/20/13.

#include "BiquadFilterNode.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeInputType.hpp"

namespace dinahmoe {
namespace audioengine {

using namespace DspBasics;

BiquadFilterNode::BiquadFilterNode(AudioContext* context, DspBasics::BiquadFilterType type_, float cutoff_, float Q_, float gain_) :
  AudioNode(context),
  m_filter(context->getSampleRate(), 2, type_, 0.5F, 1.0, 1.0) ,
  m_type(type_),
  m_newType(type_) {
	addInput("Audio input", TYPE_AUDIO);
  addInput("Cutoff control", TYPE_HYBRID);
  addInput("Q control", TYPE_HYBRID);
  addInput("Gain control", TYPE_HYBRID);
  m_inputs[1]->setInitialValue(cutoff_);
  m_inputs[2]->setInitialValue(Q_);
  m_inputs[3]->setInitialValue(gain_);
  m_filter.recomputeCoefficients(m_type, cutoff_, Q_, gain_);
  addOutput(1);
}

BiquadFilterNode::~BiquadFilterNode() {

}

void BiquadFilterNode::processInternal(int numSamples, int outputRequesting) {
  static float* inputs[2];
  static float* outputs[2];
  static bool isCutoffSampleAccurate,isQSampleAccurate,isGainSampleAccurate ;
  
  // assume that if one channels is silent, also the other will be. this can be not true
  // when using channelmergers
  if (m_inputBuffers[0]->isSilent || !(m_outputBuffers[0]->usedChannels)) {
    m_outputBuffers[0]->isSilent = true;
    return;
  } else {
    m_outputBuffers[0]->isSilent = false;
  }
  isCutoffSampleAccurate = m_inputs[1]->isSampleAccurate();
  isQSampleAccurate = m_inputs[2]->isSampleAccurate();
  isGainSampleAccurate = m_inputs[3]->isSampleAccurate();
  // if the filter type has changed, 
  if (m_type != m_newType || isCutoffSampleAccurate || isQSampleAccurate || isGainSampleAccurate ||
    m_filter.getCutoff() !=  m_inputs[1]->getValue() ||
    m_filter.getQ() !=  m_inputs[2]->getValue() ||
    m_filter.getGain() !=  m_inputs[3]->getValue()) {
    // even if a new UI event would change the value of m_newType between the branch and the assignment
    // that would be irrelevant.
    m_type = m_newType;
    // we get just the last value of the timeline, it would be really CPU expensive to be sample
    // accurate in the filter. if the AUDIO values are AUDIO, we get just the left channel
    m_filter.recomputeCoefficients(m_type,
      isCutoffSampleAccurate  ? *(m_inputBuffers[1]->data[0] + numSamples) : m_inputs[1]->getValue(),
      isQSampleAccurate  ? *(m_inputBuffers[2]->data[0] + numSamples) : m_inputs[2]->getValue(),
      isGainSampleAccurate  ? *(m_inputBuffers[3]->data[0] + numSamples) : m_inputs[3]->getValue());
  }
  size_t currentChannels = m_outputBuffers[0]->usedChannels;
  m_filter.setChannels(currentChannels);
  for (size_t chan = 0; chan < currentChannels; ++chan) {
    inputs[chan] = m_inputBuffers[0]->data[chan];
    outputs[chan] = m_outputBuffers[0]->data[chan];
  }
  m_filter.process(inputs, outputs, numSamples);
}

} //AUdioEngine
} //DMAF
