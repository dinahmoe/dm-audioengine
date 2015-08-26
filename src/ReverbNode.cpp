//
//  ReverbNode.cpp
//  Dmaf OSC Utilities
//
//  Created by Thoren Horstmann on 26.08.15.
//
//

#include "ReverbNode.h"
#include "AudioBufferSourceNode.hpp"


dinahmoe::audioengine::ReverbNode::ReverbNode(AudioContext* context) : AudioNode(context){
  addInput("Audio Input", TYPE_AUDIO);
  
  m_input = context->createGainNode();
  
  m_delays = new RefCounted<DelayNode>[4];
  m_delays[0] = context->createDelayNode(29);
  m_delays[0]->connect(m_input.get());
  m_delays[1] = context->createDelayNode(37);
  m_delays[1]->connect(m_input.get());
  m_delays[2] = context->createDelayNode(44);
  m_delays[2]->connect(m_input.get());
  m_delays[3] = context->createDelayNode(50);
  m_delays[3]->connect(m_input.get());

  
  m_sum = context->createSummingNode();
  m_sum->connect(m_delays[0].get());
  m_sum->connect(m_delays[1].get());
  m_sum->connect(m_delays[2].get());
  m_sum->connect(m_delays[3].get());


  m_allPass = new RefCounted<BiquadFilterNode>[2];
  m_allPass[0] = context->createBiquadFilterNode(DspBasics::BiquadFilterType::ALLPASS,0.5F,1.0F,1.0F);
  m_allPass[0]->connect(m_sum.get());
  m_allPass[1] = context->createBiquadFilterNode(DspBasics::BiquadFilterType::ALLPASS,0.5F,1.0F,1.0F);
  m_allPass[1]->connect(m_allPass[0].get());

  m_output = context->createGainNode();
  m_output->connect(m_allPass[1].get());

  addOutput(1);
}

dinahmoe::audioengine::ReverbNode::~ReverbNode(){}

