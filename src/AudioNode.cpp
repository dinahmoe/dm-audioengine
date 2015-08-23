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
 
#include "AudioContext.hpp"
#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include "AudioNodeOutput.hpp"
#include "Log.h"

namespace dinahmoe {
namespace audioengine {

int AudioNode::RunningId = 0;

AudioNode::AudioNode(AudioContext* context) :
  m_context(context),
  m_state(NEW),
  m_canProcess(false),
	m_lastProcessingTime(-1),
  m_normalRefCount(0),
  m_connectionRefCount(0),
  m_grabberFn(nullptr),
  m_newGrabberFn(nullptr),
  m_grabberFnArgs(nullptr),
  m_newGrabberFnArgs(nullptr),
  m_grabberUpdated(false),
  m_requestToExecuteOnProcess(false),
  m_requestCompleted(nullptr) {
	ID = RunningId++;
}

AudioNode::~AudioNode() {
  for (size_t i = 0; i < m_inputs.size(); ++i) {
    delete m_inputs[i];
  }
  for (size_t i = 0; i < m_outputs.size(); ++i) {
    delete m_outputs[i];
    delete m_outputBuffers[i];
  }
  //dmaf_log(Log::Debug, "References: Node %d deleted: refs %d connections %d ", ID, m_normalRefCount.load(), m_connectionRefCount.load());
}

void AudioNode::addInput(const char* name_, const AudioNodeInputType& type_) {
  m_inputs.push_back(new AudioNodeInput(this, name_, type_, m_inputs.size()));
  m_inputBuffers.resize(m_inputs.size());
  inputChannels.push_back(0);
}

void AudioNode::addOutput(unsigned int channelCount) {
  m_outputs.push_back(new AudioNodeOutput(this, m_outputs.size(), channelCount));
  m_outputBuffers.push_back(new AudioBuffer(context()->getOutputChannels(), m_context->getBufferSize()));
  m_outputBuffers[m_outputs.size() - 1]->setUsedChannels(channelCount);
  outputChannels.push_back(channelCount);
  m_outputHasBeenProcessed.push_back(false);
}

void AudioNode::connect(AudioNode* destination, unsigned int output, unsigned int input) {
  if (destination == NULL) {
    dm_debug_log(m_context->getLog(), Log::Error, "Trying to connect a node of type %s to a node that doesn't exist. did you create it?", this->getType());
    return;
  }

  //
  if (!canConnect(destination, output, input)) {
    return;
  }

	if (input >= destination->m_inputs.size()) {
    dm_debug_log(m_context->getLog(),Log::Error, "Trying to connect node %d(%d) to %d (%d doesn't exist)", ID, output, destination->ID, input);
		return;
	}

	if (output >= m_outputs.size()) {
    dm_debug_log(m_context->getLog(),Log::Error, "Trying to connect node %d(%d doesn't exist) to %d (%d)", ID, output, destination->ID, input);
		return;
	}
  context()->lock();

	destination->m_inputs[input]->connect(m_outputs[output]);
  m_outputs[output]->addInput(destination->m_inputs[input]);

  context()->unlock();
}

bool AudioNode::canConnect(AudioNode* destination, unsigned int output, unsigned int input) {
  return true;
}

void AudioNode::scheduleRequestOnProcess(bool* hasChanged) {
  m_requestCompleted = hasChanged;
  *m_requestCompleted = false;
  m_requestToExecuteOnProcess = true;
}

void AudioNode::process_executeRequest() {

}



void AudioNode::process_inputChannelsChanged(unsigned int inputNumber_) {
  int maxInputChannels = 0;
  if (m_inputs.size() > 0) {
    maxInputChannels = *(std::max_element(inputChannels.begin(), inputChannels.end()));
  }
  for (unsigned int i = 0; i < m_outputs.size(); ++i) {
    m_outputs[i]->setChannels(maxInputChannels);
  }
}


void AudioNode::disconnectAllOutputs() {
  context()->lock();
  for (unsigned int i = 0; i < m_outputs.size(); ++i) {
    m_outputs[i]->disconnect();
  }
  context()->unlock();
}

void AudioNode::disconnect(unsigned int output) {
  context()->lock();
	if (output > m_outputs.size()) {
    dm_debug_log(m_context->getLog(),Log::Error, "Trying to disconnect node %d(%i doesn't exist)", ID, output);
		return;
	}
	m_outputs[output]->disconnect();
  context()->unlock();
}

// There is no need to worry that the code being executed during finishDeref
// in case of *no connections* creates a race condition with this. If there
// are 0 connections it means that the variable is not referenced anywhere
// (not even in a variable) so it's impossible to increment the ref count again
void AudioNode::incrementRefCount(RefType refType) {
	if (refType == RefTypeNormal) {
		++m_normalRefCount;
	} else if (refType == RefTypeConnection) {
		++m_connectionRefCount;
	}
 // dm_debug_log(m_context->getLog(),Log::Debug, "References: Node %d referenced type %d: refs %d connections %d ", ID, refType, m_normalRefCount.load(), m_connectionRefCount.load());
}

// This method could be called both by the main thread because
// (1) a RefCounted ptr has been deleted or from the audio thread
// because (2) of a disconnect or the deletion of a node that was
// connected to us.
void AudioNode::decrementRefCount(RefType refType) {
  bool hasLock;
  if (!m_context->canUpdateGraph()) {
    m_context->lock();
    hasLock = true;
  } else {
    hasLock = m_context->tryLock();
  }

  if (hasLock) {
    finishDeref(refType);
    m_context->unlock();
  } else {
    assert(m_context->isAudioThread());
    m_context->deferFinishDeref(this);
  }
}

void AudioNode::finishDeref(RefType refType) {
	if (refType == RefTypeNormal) {
		--m_normalRefCount;
	} else if (refType == RefTypeConnection) {
		--m_connectionRefCount;
	}
  //dm_debug_log(m_context->getLog(),Log::Debug, "References: Node %d finish dereference type %d: refs %d connections %d ", ID, refType, m_normalRefCount.load(), m_connectionRefCount.load());
	if (!m_normalRefCount && !m_connectionRefCount) {
    m_state = DISAPPEARING;
    disconnectAllOutputs();
		// this will eventually defer the real delete to the main thread
		m_context->markForDeletion(this);
	}
}

void AudioNode::setOutputChannels(unsigned int outputIndex, unsigned int channelCount) {
  assert(context()->canUpdateGraph());
  //dm_debug_log(m_context->getLog(),Log::Debug, "References: Node %d set channel count %d", ID, channelCount);
  m_outputBuffers[outputIndex]->setUsedChannels(channelCount);
  outputChannels[outputIndex] = channelCount;
}


void AudioNode::startGrabbingSamples(GrabbingFunction grabberFn_, void* args_) {
  m_newGrabberFn = grabberFn_;
  m_newGrabberFnArgs = args_;
  m_grabberUpdated = true;
}

void AudioNode::stopGrabbingSamples() {
  m_newGrabberFn = nullptr;
  m_newGrabberFnArgs = nullptr;
  m_grabberUpdated = true;
}

// the process method takes care of the following:
// ensuring that the input nodes are not pulled twice
// depending on the fan-in, fan-out, assigning m_inputBuffer and m_outputBuffer
// with the correct in-place or internal pointers
// when this is setup, processInternal is called

AudioBuffer* AudioNode::process(size_t numSamples, unsigned int indexRequesting) {
	// get current timestamp
	dm_time_seconds currentTime = m_context->getCurrentTime();
	if (m_lastProcessingTime >= currentTime) {
  	// we have already pulled our inputs and processed them during this render quantum
		// the output buffers are available in m_outputBuffer
    if (m_outputHasBeenProcessed[indexRequesting]) {
      return m_outputBuffers[indexRequesting];
    }
	} else {
    m_outputHasBeenProcessed.assign(m_outputHasBeenProcessed.size(), false);
  }
  m_outputHasBeenProcessed[indexRequesting] = true;
  m_lastProcessingTime = currentTime;
  
  if (m_state == DISAPPEARING) {
    m_outputBuffers[indexRequesting]->zero();
    return m_outputBuffers[indexRequesting];
  }
  
  for (unsigned int i = 0; i < m_inputs.size(); ++i) {
    // these buffers are read only because they could be the one cached in the
    // output nodes, and they must be just an input to processing internal
    const AudioBuffer* tmp = m_inputs[i]->pull(numSamples);
    m_inputBuffers[i] = tmp;
  }

  if (m_requestToExecuteOnProcess) {
    process_executeRequest();
    m_requestToExecuteOnProcess = false;
    if (m_requestCompleted != nullptr)
      *m_requestCompleted = true;
  }
  processInternal(numSamples, indexRequesting);
  if (m_grabberUpdated) {
    m_grabberFn = m_newGrabberFn;
    m_grabberFnArgs = m_newGrabberFnArgs;
    m_grabberUpdated = false;
  }
  if (m_grabberFn) {
    const float* temp[2];
    for (unsigned int i = 0; i < m_outputBuffers[indexRequesting]->usedChannels; ++i) {
      temp[i] = m_outputBuffers[indexRequesting]->getChannelData(i);
    }
    m_grabberFn(m_grabberFnArgs, temp,m_outputBuffers[indexRequesting]->usedChannels, numSamples);
  }


  return m_outputBuffers[indexRequesting];
}
AudioNodeInput* AudioNode::getInput(const char* name_) const {
  for (unsigned int i = 0; i < m_inputs.size(); ++i) {
    if (strcmp(m_inputs[i]->getName(), name_) == 0) {
      return m_inputs[i];
    }
  }
  dm_debug_log(m_context->getLog(),Log::Error, "Node %d has no input named %s", ID, name_);
  assert(false);
  return NULL;
}

} // AudioEngine
} // DMAF
