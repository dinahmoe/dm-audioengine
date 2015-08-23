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

#include "DynamicsCompressorNode.hpp"
#include "AudioContext.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeOutput.hpp"

namespace dinahmoe {
namespace audioengine {
  
DynamicsCompressorNode::DynamicsCompressorNode(AudioContext* context) :
  AudioNode(context) {
  addInput("Audio input", TYPE_AUDIO);
  addInput("Sidechain input", TYPE_AUDIO);
  addInput("threshold", TYPE_CONTROL);
  addOutput(1);
  this->Compressor::initialize(m_context->getSampleRate(), m_context->getOutputChannels(), m_context->getBufferSize());
  m_temp.resize(m_context->getOutputChannels(), m_context->getBufferSize());
}

DynamicsCompressorNode::~DynamicsCompressorNode() {

}

// again a NOP: when we attach with a  source to one of the inputs,
// we should avoid the default behaviour of having the used output channels
// to be assigned to 1: the need to remain in the starting configuration
void DynamicsCompressorNode::process_inputChannelsChanged(unsigned int inputNumber_) {
  if (inputNumber_ == 0) {
    for (int i = 0; i < m_outputs.size(); ++i) {
      m_outputs[i]->setChannels(inputChannels[0]);
    }
    this->Compressor::SetChannels(inputChannels[0]);
  }
}

void DynamicsCompressorNode::processInternal(int numSamples, int outputRequesting) {
  const AudioBuffer& signal = *m_inputBuffers[0];
  const AudioBuffer& sidechain = *m_inputBuffers[1];
  AudioBuffer& output = *m_outputBuffers[0];
  
  if (m_inputBuffers[1]->isSilent) {
    this->Compressor::FeedZeroSignal();
  } else {
    this->Compressor::FeedDriveSignal(sidechain, m_temp);
    m_temp.isSilent = false;
  }
  if (signal.isSilent) {
    m_outputBuffers[0]->isSilent = true;
    return;
  } else {
    this->Compressor::Process(signal, output, m_temp);
    m_outputBuffers[0]->isSilent = false;
  }
}


}
}
