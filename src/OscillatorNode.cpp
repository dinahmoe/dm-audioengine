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
 
//  Created by Alessandro Saccoia on 2/26/14.

#include "OscillatorNode.hpp"
#include "AudioNodeInput.hpp"

#include <functional>
#include <iostream>

namespace dinahmoe {
namespace audioengine {

OscillatorNode::OscillatorNode(AudioContext* context, dsp::Oscillator::WaveformType type_, float frequency_, float phase_) :
  AudioNode(context),
  m_oscillator(context->getSampleRate(), type_),
  m_playing(false),
  m_time(0){
  addInput("Frequency control", TYPE_HYBRID);
  m_oscillator.setPhase(phase_);
  m_inputs[0]->setInitialValue(frequency_);
	addOutput(1);
}

OscillatorNode::~OscillatorNode() {
	
}
 
void OscillatorNode::processInternal(int numSamples, int outputRequesting) {
  if(m_time <= m_context->getCurrentTime() && m_time != 0){
    if(m_playing){
      m_playing = false;
    }else{
      m_playing = true;
    }
    m_time = 0;
  }
  if(m_playing){
  float* out =  m_outputBuffers[0]->data[0];
  m_outputBuffers[0]->isSilent = false;
  if (m_inputs[0]->isSampleAccurate()) {
    if (!m_inputBuffers[0]->isSilent) {
      float *pFreq = m_inputBuffers[0]->data[0];
      std::generate(
        out,
        out + numSamples,
        [&]() { return m_oscillator.nextSample(*(pFreq++)); }
      );
//      for (int i = 0; i < numSamples; ++i)
//        std::cout << i << " " << out[i] << std::endl;
    } else {
      // we output DC at the current value of the oscillator!
      float value = m_oscillator.nextSample(.0F);
      std::fill(out, out + numSamples, value);
    }
    
  } else {
    float freq = m_inputs[0]->getValue();
    std::generate(out, out + numSamples, [&]() { return m_oscillator.nextSample(freq);});
  }
  }else{
    m_outputBuffers[0]->isSilent = true;
  }
}
  
  void OscillatorNode::start(dm_time_seconds time){
    if(!m_playing){
      if(time <= m_context->getCurrentTime() && time <= 0){
        m_time = 0;
        m_playing = true;
      }else{
        m_time = time;
      }
    }
  }
  void OscillatorNode::stop(dm_time_seconds time){
    if(m_playing){
      if(time <= m_context->getCurrentTime() && time <= 0){
        m_time = 0;
        m_playing = false;
      }else{
        m_time = time;
      }
    }
  }
  
} //AudioEngine
} // DMAF