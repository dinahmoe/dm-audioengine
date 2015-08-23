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

#include <cstring>
#include <iostream>

#include "AudioNodeInput.hpp"
#include "AudioNodeOutput.hpp"
#include "AudioGainNode.hpp"
#include "Log.h"
#include "InMemoryBuffer.hpp"
#include "AudioFileReader.hpp"
#include "AudioBufferSourceNode.hpp"
#include "BiquadFilterNode.hpp"
#include "DelayNode.hpp"
#include "AudioInputNode.hpp"
#include "MemoryPool.hpp"
#include "Singleton.hpp"
#include "ControlTimeline.hpp"

namespace dinahmoe {
namespace audioengine {

AudioContext::AudioContext(float sampleRate_, size_t outputChannels_, size_t bufferSize_, bool isRealtime_) :
	m_isRunning(false),
  m_currentTime(.0f),
  m_currentSampleFrames(0),
  m_sampleRate(sampleRate_),
  m_outputChannels(outputChannels_),
  m_bufferSize(bufferSize_),
  m_isRealtime(isRealtime_),
  m_log(new dinahmoe::Log()),
  m_gc(m_log, 50),
  m_currentTimeslice(.0f),
  m_currentPoliphony(0),
  m_currentBufferProcessingTime(.0F),
  m_currentBufferProcessingCpu(.0F) {

  m_log->SetLevel(Log::Debug);

  // initialize pools, they will be used right away
  // note: these pools can be accessed just by the realtime thread
  m_audioNodesPool.initialize(m_log.get(), 1024);
  m_audioNodeOutputsPool.initialize(m_log.get(), 1024);
  m_audioNodeInputsPool.initialize(m_log.get(), 1024);
  m_audioNodesToDeletePool.initialize(m_log.get(), 1024);

  registerAsMainThread();
  m_masterGainNode = new AudioGainNode(this, 1.0F);
  m_osAudioInput = new AudioInputNode(this);
  m_audioInputGain = new AudioGainNode(this, 1.0F);
  // even if we have a raw pointer, consider this as a reference
  // to avoid the node to get deleted
  m_masterGainNode->incrementRefCount();
  m_audioInputGain->incrementRefCount();
  m_osAudioInput->connect(m_audioInputGain);
  m_silentBuffer = new float[m_bufferSize];
  memset((void*)m_silentBuffer, 0, m_bufferSize * sizeof(float));
  m_fileReader = new AudioFileReader();


  dm_debug_log(m_log, Log::Info, "AudioContext started");
}

float AudioContext::getSampleRate() const {
  return m_sampleRate;
}

size_t AudioContext::getOutputChannels() const {
  return m_outputChannels;
}

size_t AudioContext::getBufferSize() const {
  return m_bufferSize;
}

AudioContext::~AudioContext() {
	if (m_isRunning) {
    dmaf_log(Log::Warning, "Please call stop() to deinitialize the library gracefully");
    assert(false);
  }

  // The AudioContext needs to be the last thing that is
  // deleted in the framework.
  // all the nodes that need to be deleted will
  // end up being having lost every reference
  m_masterGainNode->decrementRefCount();
  m_audioInputGain->decrementRefCount();
  m_osAudioInput->decrementRefCount();

  delete [] m_silentBuffer;

  dmaf_log(Log::Debug, "AudioContext deleted");
}

AudioGainNode* AudioContext::masterGainNode() {
  return m_masterGainNode;
}

AudioGainNode* AudioContext::masterInputGain() {
  return m_audioInputGain;
}

AudioInputNode* AudioContext::masterInputNode() {
  return m_osAudioInput;
}


void AudioContext::start() {
  if (m_isRunning) {
    dmaf_log(Log::Warning, "The DMAF is already started");
  }
  m_gc.Start();
  m_isRunning = true;
}

void AudioContext::stop() {
  if (!m_isRunning) {
    dmaf_log(Log::Warning, "The DMAF is already stopped");
  }
  m_isRunning = false;
  m_gc.Stop();
}
  // IMPORTANT: if we are still initializing the AudioEngine from
  // the scheduler, or from whichever other interface will setup and
  // operate on the AudioEngine in the future, we don't want
  // the initial creation of the processing chain (computationally expensive)
  // to stop and probably make the audio glitch when this is first activated.
  // An external audio processing library could even use a watchdog and
  // complain about this. So if the audiocontext has been initialized but
  // not activated it's the right moment to add all the synths and effects
  // because all the actions will be made from the main scheduling thread.
  // There could be a callback triggered at the end of the init action
  // to signal that the initialization has been completed and the
  // realtime thread can start. One thing to remember is that the
  // thread for the asynchronous file loading should be started in the
  // initialization because we normally allocate buffers in the init.

void AudioContext::markForDeletion(AudioNode* node_) {
  //node_->releaseNodeInputsToPool();
  *(m_audioNodesToDeletePool.pushBackInList(m_audioNodesToDelete)) = node_;
}

void AudioContext::markForUpdate(AudioNodeInput* nodeInput) {
  if (!canUpdateGraph()) { //defer
    lock();
    *(m_audioNodeInputsPool.pushBackInList(m_dirtyAudioNodeInputs)) = nodeInput;
    unlock();
  } else {
    nodeInput->updateInternalState();
  }
}

void AudioContext::markForUpdate(AudioNodeOutput* nodeOutput) {
  if (!canUpdateGraph()) { //defer
    lock();
    *(m_audioNodeOutputsPool.pushBackInList(m_dirtyAudioNodeOutputs)) = nodeOutput;
    unlock();
  } else {
    nodeOutput->updateInternalState();
  }
}

// called by the audiodestination at the end of each render quantum
void AudioContext::updateInternalState() {
	if (tryLock()) {
    if (m_finishedSourceNodes.size() > 0) {
      for (auto fsn: m_finishedSourceNodes) {
        fsn->decrementRefCount();
      }
      m_audioNodesPool.returnToPool(m_finishedSourceNodes, m_finishedSourceNodes.begin(), m_finishedSourceNodes.end());
		}

    for (auto no: m_dirtyAudioNodeOutputs) {
      no->updateInternalState();
    }
    m_audioNodeOutputsPool.returnToPool(m_dirtyAudioNodeOutputs, m_dirtyAudioNodeOutputs.begin(), m_dirtyAudioNodeOutputs.end());

    for (auto ni: m_dirtyAudioNodeInputs) {
      ni->updateInternalState();
    }
    m_audioNodeInputsPool.returnToPool(m_dirtyAudioNodeInputs, m_dirtyAudioNodeInputs.begin(), m_dirtyAudioNodeInputs.end());

    for (auto nd: m_nodesNeedingDeref) {
      nd->finishDeref(AudioNode::RefTypeNormal);
    }
    m_audioNodesPool.returnToPool(m_nodesNeedingDeref, m_nodesNeedingDeref.begin(), m_nodesNeedingDeref.end());

		if (m_audioNodesToDelete.size()) {
      if (m_gc.TryLock()) {
        // the items in m_audioNodesToDelete come from a fixed allocated pool of pointers and *MUST BE returned to it!*
        // this is because m_audioNodesToDelete is just accessed by the audio thread.
        // So PushGarbage (that is protected by m_gc.TryLock()) actually copies in the other pool in the gargabe collector.
        // The gargabe collector will delete (actually give them back to the memory pool) the nodes and return the pointers to the
        // DmEasyPool
        m_gc.PushNodesGarbage(m_audioNodesToDelete.begin(), m_audioNodesToDelete.end());
        m_audioNodesToDeletePool.returnToPool(m_audioNodesToDelete, m_audioNodesToDelete.begin(), m_audioNodesToDelete.end());
        
        // m_controlEventsToDelete contains also pointers: they have been originally allocated from the Main Thread, user code,
        // in the m_newEvent lists of the various Timelines. The Process thread has spliced them in m_events or to m_controlEventsToDelete.
        // the garbage collector thread is the one responsible of clearing this list of pointers.
        m_gc.PushControlGarbage(m_controlEventsToDelete);
        m_gc.Unlock();
      }
    }

		unlock();
	}
}


void AudioContext::sourceNodeCreated(AudioNode* node) {
  //dmaf_log(Log::Debug, "References: Source node %i referenced", node->ID);
  node->incrementRefCount();
  *(m_audioNodesPool.pushBackInList(m_activeSourceNodes)) = node;
}

void AudioContext::sourceNodeCompleted(AudioNode* node) {
  assert(canUpdateGraph());
  std::list<AudioNode*>::iterator nodeIt = std::find(m_activeSourceNodes.begin(), m_activeSourceNodes.end(), node);
  assert(nodeIt != m_activeSourceNodes.end());
  m_finishedSourceNodes.splice(m_finishedSourceNodes.end(), m_activeSourceNodes, std::find(m_activeSourceNodes.begin(), m_activeSourceNodes.end(), node));
}
// crash when suspending: the m_activeSourceNodes gets called, but still there are a lot of
// SourceNodes attached to the processing graph. solution: setting their state to finished
// in the next rendering quantum.
// note that this gets called by the main thread if we are stopping the rendering,
// and at this point the driver will be already stop and JOINED so it's safe to operate
// on the audio context. If there will be the need of a PANIC method, the main thread should
// raise a flag so that stopSourceNodes will be called from the updateInternal state itself!
// the new crash is because there are still references to the sounds in the Dmaf framework, so these
// nodes do not get really deleted and the graph is not recomputed.
void AudioContext::stopAllSourceNodes() {
  m_finishedSourceNodes.splice(m_finishedSourceNodes.end(), m_activeSourceNodes);
}


// creation methods
InMemoryBuffer* AudioContext::createInMemoryBuffer(std::string path) {
    InMemoryBuffer* toReturn = new InMemoryBuffer();
    m_fileReader->decodeAudioFileSync(path, toReturn);
    return toReturn;
}

RefCounted<AudioGainNode> AudioContext::createGainNode(float gain_) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<AudioGainNode, std::mutex>>::instance().acquire();
  return RefCounted<AudioGainNode>(new (memoryPtr) AudioGainNode(this, gain_));
}

RefCounted<BiquadFilterNode> AudioContext::createBiquadFilterNode(DspBasics::BiquadFilterType type_, float cutoff_,float q_, float gain_) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<BiquadFilterNode, std::mutex>>::instance().acquire();
  return RefCounted<BiquadFilterNode>(new (memoryPtr) BiquadFilterNode(this, type_, cutoff_, q_, gain_));
}

RefCounted<DelayNode> AudioContext::createDelayNode(float initialDelay, float maxDelay) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<DelayNode, std::mutex>>::instance().acquire();
  return RefCounted<DelayNode>(new (memoryPtr) DelayNode(this, maxDelay, initialDelay));
}

RefCounted<AudioBufferSourceNode> AudioContext::createAudioBufferSourceNode(float playbackSpeed_) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<AudioBufferSourceNode, std::mutex>>::instance().acquire();
  AudioBufferSourceNode* node = new (memoryPtr) AudioBufferSourceNode(this, playbackSpeed_);
  lock();
  sourceNodeCreated(node);
  unlock();
  return RefCounted<AudioBufferSourceNode>(node);
}

RefCounted<WaveShaperNode> AudioContext::createWaveShaperNodeNode() {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<WaveShaperNode, std::mutex>>::instance().acquire();
  return RefCounted<WaveShaperNode>(new (memoryPtr) WaveShaperNode(this));
}

RefCounted<OscillatorNode> AudioContext::createOscillatorNode(float frequency_) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<OscillatorNode, std::mutex>>::instance().acquire();
  return RefCounted<OscillatorNode>(new (memoryPtr) OscillatorNode(this, dsp::Oscillator::SINE, frequency_));
}


RefCounted<SummingNode> AudioContext::createSummingNode(float initialValue1_, float initialValue2_) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<SummingNode, std::mutex>>::instance().acquire();
  return RefCounted<SummingNode>(new (memoryPtr) SummingNode(this, initialValue1_, initialValue2_));
}

RefCounted<ChannelSplitterNode> AudioContext::createChannelSplitterNode(int outputChannels) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<ChannelSplitterNode, std::mutex>>::instance().acquire();
  return RefCounted<ChannelSplitterNode>(new (memoryPtr) ChannelSplitterNode(this, outputChannels));
}

RefCounted<ChannelMergerNode> AudioContext::createChannelMergerNode(int inputChannels) {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<ChannelMergerNode, std::mutex>>::instance().acquire();
  return RefCounted<ChannelMergerNode>(new (memoryPtr) ChannelMergerNode(this, inputChannels));
}

RefCounted<DynamicsCompressorNode> AudioContext::createDynamicsCompressorNode() {
  void* memoryPtr = utils::Singleton<utils::MemoryPool<DynamicsCompressorNode, std::mutex>>::instance().acquire();
  return RefCounted<DynamicsCompressorNode>(new (memoryPtr) DynamicsCompressorNode(this));
}

TimelineEvent* AudioContext::createTimelineEvent() {
  return (TimelineEvent*)utils::Singleton<utils::MemoryPool<TimelineEvent, std::mutex, 4096>>::instance().acquire();
}

void AudioContext::audioCallback(float** inPlaceInputBuffer, size_t channelsIn_,
    float** inPlaceOutputBuffer, size_t channelsOut_,
    size_t numberOfFrames) {
  assert(numberOfFrames == m_bufferSize);
  if (m_isRealtime || m_isRunning) {
    std::chrono::system_clock::time_point startProcessing = std::chrono::system_clock::now();
  
    m_audioThreadId = std::this_thread::get_id();

    if (numberOfFrames == 0) {
      return;
    }

    updateInternalState();

    if (channelsIn_ > 0) {
      m_osAudioInput->setBuffers(inPlaceInputBuffer, channelsIn_);
    }

    if (channelsOut_ > 0) {
      AudioBuffer* outputBuffer = m_masterGainNode->process(numberOfFrames, 0);
      if (outputBuffer->isSilent) {
        for (size_t chan = 0; chan < channelsOut_; ++ chan) {
          std::fill(inPlaceOutputBuffer[chan], inPlaceOutputBuffer[chan] + numberOfFrames, 0);
        }
      } else {
        for (size_t i = 0; i < channelsOut_; ++i) {
          float *pIn = outputBuffer->data[outputBuffer->usedChannels >> 1 & i];
          float *pTo = inPlaceOutputBuffer[i];
          std::copy(pIn, pIn + numberOfFrames, pTo);
        }
        #ifdef CHECK_CLIPPING
        if (!outputBuffer->isSilent) {
          for (size_t ch = 0; ch < channelsOut_; ++ch) {
            bool hasClipped = false;
            for (size_t sample = 0; sample < numberOfFrames; ++sample) {
              if (fabs(inPlaceOutputBuffer[ch][sample]) >= 1.0) {
                hasClipped = true;
                break;
              }
            }
            if (hasClipped)
              dmaf_log(Log::Error, "Clipping on channel %i", ch);
          }
        }
        #endif
      }
    }

    m_currentTimeslice = (float)numberOfFrames / m_sampleRate;
    m_currentTime += m_currentTimeslice;


    m_currentSampleFrames += numberOfFrames;
    
    std::chrono::system_clock::time_point endProcessing = std::chrono::system_clock::now();
    
    m_currentBufferProcessingTime =
      std::chrono::duration_cast<std::chrono::microseconds>(endProcessing - startProcessing).count() / 1000000.F;
    
    m_currentBufferProcessingCpu = m_currentBufferProcessingTime / m_currentTimeslice;
  }
}

void AudioContext::executeScheduledActions() {
  // loop in the fifo queue and look for stuff that will happen during this process call or before
//  action->execute(m_currentTime);
}

AudioContext::AudioContextGG::AudioContextGG(std::shared_ptr<Log> log_, unsigned int periodMs_)
  : utils::DmTimedConsumerMT(periodMs_)
  , m_log(log_) {
  m_audioNodesPool.initialize(m_log.get(), 1024);
  m_controlPool.initialize(m_log.get(), 8192);
}

void AudioContext::AudioContextGG::PushNodesGarbage(typename std::list<AudioNode*>::iterator garbageBegin, typename std::list<AudioNode*>::iterator garbageEnd) {
  while (garbageBegin != garbageEnd) {
    *(m_audioNodesPool.pushBackInList(m_nodesGarbageList)) = *garbageBegin;
    ++garbageBegin;
  }
}

void AudioContext::AudioContextGG::PushControlGarbage(std::list<TimelineEvent*>& controlGarbage_) {
  m_controlGarbageList.splice(m_controlGarbageList.end(),  controlGarbage_, controlGarbage_.begin(), controlGarbage_.end());
}

void AudioContext::AudioContextGG::TimedOperation() {
  for (auto item: m_nodesGarbageList) {
    const char* nodeType = item->getType();
    if (strcmp("AudioBufferSourceNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<AudioBufferSourceNode, std::mutex>>::instance().release((AudioBufferSourceNode*)item);
    } else if (strcmp("AudioGainNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<AudioGainNode, std::mutex>>::instance().release((AudioGainNode*)item);
    } else if (strcmp("AudioInputNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<AudioInputNode, std::mutex>>::instance().release((AudioInputNode*)item);
    } else if (strcmp("BiquadFilterNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<BiquadFilterNode, std::mutex>>::instance().release((BiquadFilterNode*)item);
    } else if (strcmp("ChannelMergerNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<ChannelMergerNode, std::mutex>>::instance().release((ChannelMergerNode*)item);
    } else if (strcmp("ChannelSplitterNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<ChannelSplitterNode, std::mutex>>::instance().release((ChannelSplitterNode*)item);
    } else if (strcmp("DynamicsCompressorNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<DynamicsCompressorNode, std::mutex>>::instance().release((DynamicsCompressorNode*)item);
    } else if (strcmp("OscillatorNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<OscillatorNode, std::mutex>>::instance().release((OscillatorNode*)item);
    } else if (strcmp("SummingNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<SummingNode, std::mutex>>::instance().release((SummingNode*)item);
    } else if (strcmp("WaveShaperNode", nodeType) == 0) {
      utils::Singleton<utils::MemoryPool<WaveShaperNode, std::mutex>>::instance().release((WaveShaperNode*)item);
    } else {
      assert("Forgot a node?" && false);
    }
  }
  m_audioNodesPool.returnToPool(m_nodesGarbageList, m_nodesGarbageList.begin(), m_nodesGarbageList.end());
  
  utils::Singleton<utils::MemoryPool<TimelineEvent, std::mutex, 4096>>::instance().lock();
  for (auto item: m_controlGarbageList) {
    utils::Singleton<utils::MemoryPool<TimelineEvent, std::mutex, 4096>>::instance().releaseWithExternalLock((TimelineEvent*)item);
  }
  utils::Singleton<utils::MemoryPool<TimelineEvent, std::mutex, 4096>>::instance().unlock();
  
  m_controlGarbageList.clear();
}


} // AudioEngine
} // DMAF
