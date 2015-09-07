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
 * \file AudioContext.hpp
 *
 * \author Alessandro Saccoia
 * \date 8/23/12
 *
 */

#ifndef __AUDIOCONTEXT_HPP__
#define __AUDIOCONTEXT_HPP__

#include <vector>
#include <thread>

#include <unordered_set>
#include "RefCounted.hpp"
#include "AudioBuffer.hpp"
#include "InMemoryBuffer.hpp"
#include "Log.h"
#include "BiquadFilter.hpp"

#include "AudioNode.hpp"
#include "AudioInputNode.hpp"
#include "AudioGainNode.hpp"
#include "AudioBufferSourceNode.hpp"
#include "BiquadFilterNode.hpp"
#include "DelayNode.hpp"
#include "ChannelSplitterNode.hpp"
#include "ChannelMergerNode.hpp"
#include "DynamicsCompressorNode.hpp"
#include "WaveShaperNode.hpp"
#include "OscillatorNode.hpp"
#include "SummingNode.hpp"
#include "StereoPannerNode.h"
#include "PannerNode.h"
#include "DmTimedConsumerMT.hpp"
#include "DmEasyPoolST.hpp"

#define CHECK_CLIPPING 0

namespace dinahmoe {
namespace audioengine {

class AudioFileReader;
class TimelineEvent;
class ControlTimeline;

/*! \brief The execution context of the audio framework
 * 
 * The AudioContext handles the internal state of the processing graph keeping
 * it always in a consistent state. It is also the dispatching point for 
 * everything that has to deal with time, providing a steady reference clock.
 */
class AudioContext {
public:
  /*! \brief Constructor
   *
   */
  AudioContext(float sampleRate_, size_t outputChannels_, size_t bufferSize_, bool isRealtime_);
  
  /*! \brief Destructor
   */
	~AudioContext();
	
	void start();
	void stop();
  
  inline bool isRunning() {
    return m_isRunning;
  }
	
  inline void registerAsMainThread() {
    m_mainThreadId = std::this_thread::get_id();
  }
  
	// threading
	inline bool isMainThread() {
//define this macro when using the audiocontext or audioengine
//alone to make sure that there is no concurrent access, and assert
//that the thread that has registered itself as main is always the same one.
//removing this makes it easier to initialize the dmaf that already takes care
//of proper handling of the threads
#ifdef AUDIOCONTEXT_STRICT_THREADING
		return m_mainThreadId == std::this_thread::get_id();
#else
    return !isAudioThread();
#endif
	}
	
	inline bool isAudioThread() {
		return m_audioThreadId == std::this_thread::get_id();
	}
  
  /*! \brief Returns true if the accessing thread is realtime, or if the realtime thread is not running (audio pause state)
   *
   * Used to assert many preconditions regarding threading 
   */
  inline bool canUpdateGraph() {
    return !m_isRealtime || isAudioThread() || !isRunning();
  }
	

  /*! \brief Locks the graph mutex
   *
   * This is never called from the realtime thread. Since it can be called recursively
   * we keep track of who locked it first and release it just after the first call.
   * mustReleaseLock is a sort of receipt that will be true just for the first caller
   */
	void lock() {
    if (!m_isRealtime)
      return;
    m_contextMutex.lock();
	}
	/*! \brief Tries to the graph mutex
   *
   * This is always called from the realtime thread. 
   * \sa lock
   */
	bool tryLock() {
    if (!m_isRealtime)
      return true;
    return m_contextMutex.try_lock();
	}
	
  /*! \brief Unlocks the graph mutex
   *
   * This can be called by either the realtime after a succesful trylock
   * or from the other threads after having locked the processing graph mutex.
   */
	void unlock() {
    if (!m_isRealtime)
      return;
		m_contextMutex.unlock();
	}
	
  /*! \brief Decodes an audiofile synchronously 
   *
   * \return an InMemoryBuffer. check isReady() to see if the decoding was sucesful
   */   
	InMemoryBuffer* createInMemoryBuffer(std::string path);
  
  /*! \brief Creates a gain node with an optional gain value
   */ 
	RefCounted<AudioGainNode> createGainNode(float gain_ = 1.0F);
  
  /*! \brief Creates a filter node
   */
  RefCounted<BiquadFilterNode> createBiquadFilterNode(
    DspBasics::BiquadFilterType type_ = DspBasics::BiquadFilterType::LOWPASS, float cutoff_ = 0.5F,float q_ = 1.0F, float gain_ = 1.0F);
  
  /*! \brief Creates a delay node
   */
  RefCounted<DelayNode> createDelayNode(float initialDelay = 0.5F, float maxDelay = 6.0F);
  
  /*! \brief Creates a gain node with an optional playbackSpeed_ value
   */ 
  RefCounted<AudioBufferSourceNode> createAudioBufferSourceNode(float playbackSpeed_ = 1.0F);

  /*! \brief Creates a channel splitter with an optional number of channels
   */ 
  RefCounted<ChannelSplitterNode> createChannelSplitterNode(int outputChannels = 2);

  /*! \brief Creates a channel merger with an optional number of channels
   */ 
  RefCounted<ChannelMergerNode> createChannelMergerNode(int inputChannels = 2);

  /*! \brief Creates a compressor
   */ 
  RefCounted<DynamicsCompressorNode> createDynamicsCompressorNode();

  /*! \brief Creates a waveshaper
   */ 
  RefCounted<WaveShaperNode> createWaveShaperNodeNode();

  /*! \brief Creates an oscillator node
   */ 
  RefCounted<OscillatorNode> createOscillatorNode(float frequency_ = 220.F);

  /*! \brief Creates a summing node
   */ 
  RefCounted<SummingNode> createSummingNode(float initialValue1_ = 1.0F, float initialValue2_ = 1.0F);
  
  /*! \brief Creates a steropanning node
   */
  RefCounted<StereoPannerNode> createStereoPannerNode(float pan = 0.0F);
  
  /*! \brief Creates a panning node
   */
  RefCounted<PannerNode> createPannerNode(AudioListener* listener, float x, float y, float z);
  
  /*! \brief Creates a TimelineEvent
   */ 
  TimelineEvent* createTimelineEvent();
  
	/*! \brief Returns the master gain node of the framework. Is the big knob
   *  of this fancy radio.
   */ 
	AudioGainNode* masterGainNode();
  
  /*! \brief Returns the master input gain node of the framework. 
   */ 
	AudioGainNode* masterInputGain();
  
   /*! \brief Returns the master input node of the framework. 
   */ 
	AudioInputNode* masterInputNode();
  
  
	
  /*! \brief Updates the processing graph applying the latest requested changes
   *
   * \pre The accessing thread can update the graph
   * \post The m_dirtyAudioNodeInputs and m_dirtyAudioNodeOutputs sets have been cleared
   */ 
	void updateInternalState();
	
	
  /*! \brief Marks this node for deletion
   *
	 * Called from the main thread, to be executed on the realtime thread
   *
   * \todo Freeing memory in the realtime thread is *bad* and should be avoided. Setup 
   * a general mechanism to defer the free to the main thread again.
   *
   */ 
	void markForDeletion(AudioNode* node);
	
  /*! \brief Updates the rendering state of an AudioNodeInput or defers the update if it can't update the graph
   */
	void markForUpdate(AudioNodeInput* nodeInput);
  
  /*! \brief Updates the rendering state of an AudioNodeOutput or defers the update if it can't update the graph
   */
	void markForUpdate(AudioNodeOutput* nodeOutput);
	

  /*! \brief Signal the context that a source node has started playing
   *
	 * source nodes call this to signal that they
	 * have started or finished processing. it's needed just to have a way to stop all the notes
	 * that are playing like when pressing "stop" in a DAW
   */ 
	void sourceNodeCreated(AudioNode* node);
  
  /*! \brief Signal the context that a source node has completed its playback
   *
   */
	void sourceNodeCompleted(AudioNode* node);
  
  
  void stopAllSourceNodes();
	
  /*! \brief Signal the context that finishDeref needs to be called on a node
   *
   * This is called from AudioNode::deref just for internal disconnections, so the 
   * type of disconnection is not needed. No needs of locking to protect the list too
   *
	 * \pre The caller thread is the AudioThread
   */ 
  void deferFinishDeref(AudioNode* node_) {
    *(m_audioNodesPool.pushBackInList(m_nodesNeedingDeref)) = node_;
  }
  
  /*! \brief returns the engine time
   *
   */
  inline dm_time_seconds getCurrentTime() {
    return float(m_currentSampleFrames) / m_sampleRate;
//    return m_currentTime;
  }
  
  /*! \brief Returns the sample rate
   *
   */
  float getSampleRate() const;

  /*! \brief Returns the number of channels for this context
   *
   */
  size_t getOutputChannels() const;
  
  /*! \brief Returns the buffer size
   *
   */
  size_t getBufferSize() const;
  
  inline long unsigned int getCurrentSampleFrames() {
    return m_currentSampleFrames;
  }
  
  /*! \brief The audio hardware calls here periodically to gets samples
   *
   * This is the place to ask for sample in offline processing
   *
   * \param inPlaceInputBuffer a pointer to 2 buffers o floats pre allocated
   * \param channelsIn the number of input channels
   * \param inPlaceOutputBuffer a pointer to 2 buffers o floats pre allocated
   * \param channelsout the number of output channels
   * \param numberOfFrames the number of frames requested
   */
  void audioCallback(float** inPlaceInputBuffer, size_t channelsIn_,
    float** inPlaceOutputBuffer, size_t channelsOut_,
    size_t numberOfFrames);
  
  /*! \brief Utility method to get a zeroed buffer of the right size
   */
  float* const getSilentBuffer() const {
    return m_silentBuffer;
  }
  
  dinahmoe::Log* getLog() {
    return m_log.get();
  }
  
  std::list<TimelineEvent*>& getControlEventsToDeleteList() { return m_controlEventsToDelete; }
  
  utils::DmEasyPoolST<AudioNode*>& getAudioNodesPool() {return m_audioNodesPool;}
  utils::DmEasyPoolST<AudioNodeOutput*>& getAudioNodeOutputsPool() { return m_audioNodeOutputsPool; };
  utils::DmEasyPoolST<AudioNodeInput*>& getAudioNodeInputsPool() { return m_audioNodeInputsPool; }
  
  
  
private:

  /** A garbage collector specialized for the audionode
   */
  class AudioContextGG :
    public utils::DmTimedConsumerMT {
  public:
    AudioContextGG(std::shared_ptr<Log> log_, unsigned int periodMs_);
    void PushNodesGarbage(typename std::list<AudioNode*>::iterator garbageBegin, typename std::list<AudioNode*>::iterator garbageEnd);
    void PushControlGarbage(std::list<TimelineEvent*>& controlGarbage_);
    void TimedOperation();
  private:
    std::shared_ptr<Log> m_log;
    std::list<AudioNode*> m_nodesGarbageList;
    std::list<TimelineEvent*> m_controlGarbageList;
    utils::DmEasyPoolST<AudioNode*> m_audioNodesPool;
    utils::DmEasyPoolST<TimelineEvent*> m_controlPool;
  };

  AudioContext();
  	
  void executeScheduledActions();
  
  
  /*! \brief This is just a guard to check that init() is called before doing anything
   */
	bool m_isInitialized;
  
  
  /*! \brief This is a guard that looks too similar to m_isInitialized
   *
   * /todo make initialization less cumberstome
   */
	bool m_isRunning;

	AudioGainNode* m_masterGainNode;
  
  AudioInputNode* m_osAudioInput;
	AudioGainNode* m_audioInputGain;
	
	// inserting one node here increments its internal reference count
	// good for source nodes that are playing. Called from the noteOn 
	// method in case of a buffersourcenode
	std::list<AudioNode*> m_activeSourceNodes;
  std::list<AudioNode*> m_finishedSourceNodes;
	std::list<AudioNode*> m_audioNodesToDelete;
	std::list<TimelineEvent*> m_controlEventsToDelete;
  
  // pools
  utils::DmEasyPoolST<AudioNode*> m_audioNodesPool;
  utils::DmEasyPoolST<AudioNodeOutput*> m_audioNodeOutputsPool;
  utils::DmEasyPoolST<AudioNodeInput*> m_audioNodeInputsPool;
  utils::DmEasyPoolST<AudioNode*> m_audioNodesToDeletePool;
	
	// use a set for performance reasons: since some connections or node
	// deletions can trigger multiple inserts in the graph, we want to 
	// make sure that there is just one copy of the element in the set so
	// that the connections are rebuilt just once
	std::list<AudioNodeInput*> m_dirtyAudioNodeInputs;
	std::list<AudioNodeOutput*> m_dirtyAudioNodeOutputs;
	
  
  std::list<AudioNode*> m_nodesNeedingDeref;
  
	// threading
	std::thread::id m_mainThreadId;
  std::thread::id m_audioThreadId;
  
  /*! \brief The main and only mutex
   */
  std::recursive_mutex m_contextMutex;
  
  /*! \brief Our time in seconds
   */
  dm_time_seconds m_currentTime;
  
  long unsigned int m_currentSampleFrames;

  float m_sampleRate;
  size_t m_outputChannels;
  size_t m_bufferSize;
  
  /*! \brief An asynchronous file reader. It works but it's not used right now
   */ 
  AudioFileReader* m_fileReader;
  
  bool m_isRealtime;
  
  float* m_silentBuffer;
  
  std::shared_ptr<dinahmoe::Log> m_log;
  
  AudioContextGG m_gc;

 public:
  /** The duration of the current timeslice in seconds */
  dm_time_seconds m_currentTimeslice;
  
  // statistics
  volatile int m_currentPoliphony;
  volatile float m_currentBufferProcessingTime;
  volatile float m_currentBufferProcessingCpu;
  
  
}; //AudioContext



} //AudioEngine
} //DMAF

#endif

/* 	alex TODOs
	enforce constness
	create mechanism for automatic pull nodes: these nodes have no connections but are
	still referenced, so they don't get pulled. A node when it's first created adds itself
	to this list, then every time a connection is made or the node is dereferenced with a
	refTypeConnection argument, the node adds or remove itself from the list
*/



