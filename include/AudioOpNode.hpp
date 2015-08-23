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
 
#ifndef __AudioOpNode_HPP__
#define __AudioOpNode_HPP__

#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include <functional>
#include <algorithm>

namespace dinahmoe {
namespace audioengine {

template <class OPERATION>
struct OP_TRAITS {
  
};

template <>
struct OP_TRAITS<std::plus<float>> {
  static float Identity() { return 0.F; }
};

template <>
struct OP_TRAITS<std::multiplies<float>> {
  static float Identity() { return 1.0F; }
};


template <template<class> class OPERATION>
class AudioOpNode :
	public AudioNode {
public:	
	AudioOpNode(AudioContext* context_, float value1_ = OP_TRAITS<OPERATION<float>>::Identity(), float value2_ = OP_TRAITS<OPERATION<float>>::Identity()) : AudioNode(context_) {
    addInput("Input1", value1_);
    addInput("Input2", value2_);
    addOutput(1);
  }
	~AudioOpNode() {
  
  }
  inline const char* getType() { return typeid(AudioOpNode<OPERATION>).name(); }
  
	void processInternal(int numSamples, int outputRequesting) {
    AudioBufferC<float>::transform<OPERATION>(*m_inputBuffers[0], *m_inputBuffers[1], *m_outputBuffers[0], numSamples);
  }

};

typedef AudioOpNode<std::multiplies> AudioGainNode;
typedef AudioOpNode<std::plus> AudioSummingNode;

} // AUDIOENGINE
} // DMAF

#endif