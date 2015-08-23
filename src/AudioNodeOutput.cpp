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
 
#include "AudioNodeOutput.hpp"
#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include "AudioContext.hpp"

namespace dinahmoe {
namespace audioengine {

AudioNodeOutput::AudioNodeOutput(AudioNode* node, int outputIndex, int channels) :
	m_node(node),
	m_channels(channels),
	m_outputIndex(outputIndex),
  m_futureChannels(channels) {

}
	
AudioNodeOutput::~AudioNodeOutput() {
  node()->context()->getAudioNodeInputsPool().returnToPool(m_inputs, m_inputs.begin(), m_inputs.end());
  node()->context()->getAudioNodeInputsPool().returnToPool(m_destinationInputs, m_destinationInputs.begin(), m_destinationInputs.end());
}

void AudioNodeOutput::addInput(AudioNodeInput* input) {
	if (std::find(m_destinationInputs.begin(), m_destinationInputs.end(), input) == m_destinationInputs.end()) {
//    dm_debug_log(m_node->context()->getLog(), Log::Debug, "AudioNodeOutput::addInput %p size %i %p", input, m_destinationInputs.size(), this);
    *(node()->context()->getAudioNodeInputsPool().pushBackInList(m_destinationInputs)) = input;
//    dm_debug_log(m_node->context()->getLog(),Log::Error, "AudioNodeOutput::addInput %p after, size %i %p", input, m_destinationInputs.size(), this);
    m_node->context()->markForUpdate(this);
  } else {
    dmaf_log(Log::Error, "Readding same input");
  }
}

// disconnects the output from every input that is connected to it
// this can arrive from the main thread, in locked state
// this can arrive also from the processing thread, on updateInternalState->finishderef
void AudioNodeOutput::disconnect() {
//    dm_debug_log(m_node->context()->getLog(),Log::Error, "AudioNodeOutput::disconnect before, size %i %p", m_destinationInputs.size(), this);
  for (auto di: m_destinationInputs) {
    di->disconnect(this);
  }
//  dm_debug_log(m_node->context()->getLog(),Log::Error, "AudioNodeOutput::disconnect after, size %i %p", m_destinationInputs.size(), this);
  node()->context()->getAudioNodeInputsPool().returnToPool(m_destinationInputs, m_destinationInputs.begin(), m_destinationInputs.end());
  m_node->context()->markForUpdate(this);
}

// the inputBuffer parameter is used to give the opportunity to have
// inplace processing
AudioBuffer* AudioNodeOutput::pull(size_t numSamples) {
    return m_node->process(numSamples, m_outputIndex);
}

void AudioNodeOutput::setChannels(int numberOfChannels) {
  if (numberOfChannels != m_channels) {
    m_futureChannels = numberOfChannels;
    m_node->context()->markForUpdate(this);
  }
}


void AudioNodeOutput::updateInternalState() {
  assert(node()->context()->canUpdateGraph());
  node()->context()->getAudioNodeInputsPool().returnToPool(m_inputs, m_inputs.begin(), m_inputs.end());
  if (m_destinationInputs.size()) {
//    dm_debug_log(m_node->context()->getLog(),Log::Error, "AudioNodeOutput::updateInternalState, size %i %p", m_inputs.size(), this);
    node()->context()->getAudioNodeInputsPool().pushBackInList(m_inputs, m_destinationInputs.size());
//    dm_debug_log(m_node->context()->getLog(),Log::Error, "AudioNodeOutput::updateInternalState, size %i %p", m_inputs.size(), this);
    auto sit = m_inputs.begin();
    auto dit = m_destinationInputs.begin();
    while (sit != m_inputs.end()) {
      *sit = *dit;
      ++sit, ++dit;
    }
  }
  m_channels = m_futureChannels;
  node()->setOutputChannels(m_outputIndex, m_channels);
  for (auto input: m_inputs) {
    input->updateChannelCount();
  }
}

} // AudioEngine
} // DMAF 
