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
 
 
 /*!
 * \file AudioNode.hpp
 *
 * \author Alessandro Saccoia
 * \date 8/23/12
 *
 */
 
#ifndef __AUDIONODE_HPP__
#define __AUDIONODE_HPP__

#include <vector>
#include <atomic>

#include "RefCounted.hpp"
#include "AudioBuffer.hpp"
#include "DmTypedefs.hpp"
#include "AudioNodeInputType.hpp"


namespace dinahmoe {
namespace audioengine {

class AudioContext;
class AudioNodeInput;
class AudioNodeOutput;

/*! \brief Abstract class for the AudioNodes
 *
 * The AudioNode is the basic building block of the processing graph.
 * The most important properties of an AudioNode are a set of AudioInputNode s
 * and one of AudioOutputNode s. The number and the properties of these are
 * defined in the constructor of the classese that inherit from AudioNode.
 *
 * \snippet AudioEngine/AudioGainNode.cpp Adding inputs
 *
 * A node computation' state is kept consistent by allowing just one thread to
 * work on it's properties. 
 * 
 */
class AudioNode {
friend class AudioNodeOutput;
friend class AudioNodeInput;
public:
  /*! \brief Constructor */
	AudioNode(AudioContext* context);
  
  /*! \brief Constructor */
	virtual ~AudioNode();
  
#pragma mark User code interface
  /*! \brief Connects a node's output to an input destination 
   * 
   * This connection will take place during the next call to 
   * UpdateInternalState if the AudioContext is rendering. It's
   * instantaneous if the context is not running.
   *
   * \param destination a destination AudioNode
   * \param output this node's output number
   * \param output the destination node's input number
   *
   */
	void connect(AudioNode* destination, unsigned int output = 0, unsigned int input = 0);

  
  /*! \brief Disconnect one output given its index */
	void disconnect(unsigned int output);
  
  /*! \brief Returns a AudioNodeInput given its name */
  AudioNodeInput* getInput(const char* name_) const;

#pragma mark Interface for subclassing
  /*! \brief Return the context */
  inline AudioContext* context() {
		return m_context;
	}
  /*! \brief Return the context */
  virtual const char* getType() = 0;
  
  // this function is called at the beginning of connect
  // and can be overridden to check for the possible allowed
  // configurations. if it returns false for a given node
  // the connection doesn't take place. It will be useful
  // in the case of a graphical interface to the audiocontext
  // where the GUI can call canConnect directly to check if it
  // can highlight the destination node input to give a hint
  // about the possibility of a connection. returns true by default
  virtual bool canConnect(AudioNode* destination, unsigned int output, unsigned int input);
  
  // request for the process_executeRequest function to be
  // executed at the next audioengine tick. an external thread
  // can poll on hasChanged to check when the change has been made.
  // also useful for avoid too many changes to be done in a short time
  void scheduleRequestOnProcess(bool* hasChanged);
  
  
  // this can do whatever, normally it's a NOP. gets called from the process
  // thread when the user code requests a change
  virtual void process_executeRequest();
  
  // this function allows one to reconfigure the output channel
  // based on the connection. it's called before propagating
  // the new channel count, and by default it assigns to the
  // outputs the channel of input one. 
  // it complements the work done by canConnect, and as canConnect
  // has sense just for nodes that are neither source or destination.
  // it's default behavior is a NOP.
  // Example: a gain node has a mono source. what to do if we allow
  // a stereo modulation to be applied? In this method there is the
  // choice.
  
  // it's called whenever the input nodes are changed, on the process thread.
  // the default implementation is querying all the inputs and setting the output
  // channels to the maximum of the inputs.
  // NOTE: the process call happens after, one can be sure that stuff got updated by
  // this time
  /*
  int maxInputChannels = 0;
  if (m_inputs.size() > 0) {
    maxInputChannels = *(std::max_element(inputChannels.begin(), inputChannels.end()));
  }
  for (int i = 0; i < m_outputs.size(); ++i) {
    m_outputs[i]->setChannels(maxInputChannels);
  }
  */
  
  virtual void process_inputChannelsChanged(unsigned int inputNumber_);

#pragma mark Interface handled by other internal classes. not private but to be considered private.
  
	/*! \brief Internal handling of reference counting */
	enum RefType {RefTypeNormal, RefTypeConnection};
  
  /*! \brief Called each time the AudioNode is referenced
   * 
   * This can be called because a RefCounted has been copied (RefTypeNormal)
   * or because a connection to an input has been made (RefTypeConnection) 
   */
	void incrementRefCount(RefType refType = RefTypeNormal);
  /*! \brief Called each time the AudioNode is dereferenced
   * 
   * This can be called because a RefCounted has been deleted (RefTypeNormal)
   * or because a connection to an input has been deleted (RefTypeConnection).
   * If the connection count 
   */
	void decrementRefCount(RefType refType = RefTypeNormal);
  void finishDeref(RefType refType);
  
  AudioBuffer* process(size_t numSamples, unsigned int indexRequesting);
	
  
  // the channel configuration of the inputs and outputs. The first is
  // set by the AudioNodeINput class depending on which outputs have been
  // connected to it, the second is set by the programmer of the inherited
  // class overriding the process_inputChannelsChanged method, or in the
  // case of the AudioBufferSource depending on the loaded buffer
  std::vector<size_t> inputChannels;
  std::vector<size_t> outputChannels;
  
  void setOutputChannels(unsigned int outputIndex, unsigned int channelCount);
  
  
  typedef void(*GrabbingFunction)(void*, const float**, int, int);
  
  // buffer, channels, count
  void startGrabbingSamples(GrabbingFunction grabberFn_, void* args_);
  void stopGrabbingSamples();
  
  int ID;
  static int RunningId;
  
  
	// we don't use any reference counting here: just make sure
	// that when a node is going to be deleted its inputs and
	// outputs are marked as dirty. In this way the processing
	// thread can take care of cleaning up the references to
	// these in/outs that still remain in the in/outs they were
	// connected to before being deleted
	std::vector<AudioNodeInput*> m_inputs;
	std::vector<AudioNodeOutput*> m_outputs;
	// these pointers are substituted with the inplace pointer
	// or with the internal temp buffers depending on the fan-in
	// and fan-out. The processInternal virtual call uses what is
	// in m_inputBuffers to compute what to write in m_outputBuffers
	// and then returns.
	std::vector<const AudioBuffer*> m_inputBuffers;
	std::vector<AudioBuffer*> m_outputBuffers;
  
	AudioContext* m_context;	

protected:
  // allow for implicit conversion from char*
  void addInput(const char* name_, const AudioNodeInputType& type_);
  void addOutput(unsigned int channelCount);
  
private:

  enum NodeState {
    DISAPPEARING,
    NEW,
    ACTIVE
  };
  
  NodeState m_state;

  
  
	
	virtual void processInternal(int numSamples, int outputRequesting) = 0;
  
  

  // this is used by nodes that require to load something, should be set to false
  // in the ctor and to false whenever the process call can happen
  bool m_canProcess;  
  
private:  
  /*! \brief Disconnect all outputs */
  void disconnectAllOutputs();
  
	// this variable keeps the last time process has been called.
	// when multiple nodes are connected to the outputs of this node
	// process could be called several times per rendering quantum.
	dm_time_seconds m_lastProcessingTime;
  std::vector<bool> m_outputHasBeenProcessed;
	
	// use atomic operations on these
  std::atomic<int> m_normalRefCount;
  
  // NB: this reflects the number of nodes that are connected to our inputs,
  // as we want to be deleted just when no one else depends on us for its
  // processing
  std::atomic<int> m_connectionRefCount;
  
  GrabbingFunction m_grabberFn;
  GrabbingFunction m_newGrabberFn;
  void* m_grabberFnArgs;
  void* m_newGrabberFnArgs;
  
  std::atomic<bool> m_grabberUpdated;
  
  std::atomic<bool> m_requestToExecuteOnProcess;
  volatile bool *m_requestCompleted;
  
};

} // AUdioEngine
} // DMAF

#endif
