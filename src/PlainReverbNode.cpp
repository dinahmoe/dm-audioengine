//
//  PlainReverbNode.cpp
//  Nodal
//
//  Created by Thoren Horstmann on 26.08.15.
//
//

#include "PlainReverbNode.h"
#include "AudioBufferSourceNode.hpp"
#include "AudioNodeInput.hpp"


dinahmoe::audioengine::PlainReverbNode::PlainReverbNode(AudioContext* context,float delay,float gain) : AudioNode(context){
  addInput("Audio Input", TYPE_AUDIO);
  addInput("Delay control", TYPE_HYBRID);
  m_inputs[0]->setInitialValue(delay);

  addInput("Gain control", TYPE_HYBRID);
  m_inputs[1]->setInitialValue(gain);

  m_sum = context->createSummingNode();
  m_gain = context->createGainNode(gain);
  m_delay = context->createDelayNode(delay);
  
  m_delay->connect(m_sum.get());
  m_gain->connect(m_delay.get());
  m_sum->connect(m_gain.get());

  
  addOutput(1);
}

dinahmoe::audioengine::PlainReverbNode::~PlainReverbNode(){}

void dinahmoe::audioengine::PlainReverbNode::processInternal(int numSamples, int outputRequesting){
  m_sum->m_inputs = m_inputs;
  m_outputs = m_sum->m_outputs;
}