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
 
//  Created by Alessandro Saccoia on 12/29/14.

#include "VLine.hpp"
#include "AudioNodeInput.hpp"
#include <list>
#include <iostream>

namespace dinahmoe {
namespace audioengine {

VLine::VLine(AudioContext* context_, float startValue_)
  : AudioNode(context_ ), m_currentValue(startValue_), m_destinationValue(m_currentValue) {
  addOutput(1);
}

VLine::~VLine() {
  std::cout << "Deleting VLine" << std::endl;
}

void VLine::setValueAtTime(const dm_time_seconds time_, const float value_) {
  m_destinationSample = (unsigned long long)(time_ * m_context->getSampleRate());
  m_destinationValue = value_;
}

void VLine::processInternal(int numSamples_, int outputRequesting_) {
  const unsigned long currentSample = context()->getCurrentSampleFrames();
  const float aDestinationValue = m_destinationValue;
  float m_deltaValue = aDestinationValue - m_currentValue;
  const long m_deltaSamples = m_destinationSample - currentSample;
  if (m_deltaValue != .0F && m_deltaSamples >= 0) {
    auto valueStep = m_deltaValue / m_deltaSamples;
    const long actualNumberOfSteps = std::min((long)numSamples_, m_deltaSamples);
    m_outputBuffers[0]->channelIsConstant[0] = false;
    m_outputBuffers[0]->channelConstantValue[0] = m_currentValue;
    size_t currentOutputSample = 0;
    do {
      m_currentValue += valueStep;
      m_outputBuffers[0]->getChannelData(0)[currentOutputSample] = m_currentValue;
    } while (++currentOutputSample < actualNumberOfSteps);
    // fill the first part
    do {
      m_outputBuffers[0]->getChannelData(0)[currentOutputSample] = m_currentValue;
    } while (++currentOutputSample < numSamples_);
  } else {
    m_currentValue = aDestinationValue;
    m_outputBuffers[0]->channelIsConstant[0] = true;
    m_outputBuffers[0]->channelConstantValue[0] = m_currentValue;
  }
}

}}
