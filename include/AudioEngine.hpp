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
 * \file AudioEngine.hpp
 *
 * \author Alessandro Saccoia
 * \date 7/19/12
 * \deprecated This class shouldn't be necessary anymore
 *
 * This class inherits both by AudioDriver and AudioEngine. deprecated.
 */

#ifndef __AudioEngine_hpp
#define __AudioEngine_hpp

#include <memory>
#include <functional>
#include <list>
#include <fstream>

#include "AudioContext.hpp"
#include "DmDictionary.h"
#include "NullMutex.hpp"
#include "TimeProvider.hpp"
#include "RingBuffer.hpp"
#include "AudioEngineConfig.hpp"

#if DMF_USES_OFFLINE_RENDERING
  #include "AudioDriver_audiofile.hpp"
#else
  #if DMF_USES_EXTERNAL_AUDIO
    #include "AudioDriver_external.hpp"
  #else
    #ifdef DM_PLATFORM_OSX
      #include "AudioDriver_osx.hpp"
    #elif DM_PLATFORM_ANDROID
      #include "AudioDriver_android.hpp"
    #elif DM_PLATFORM_IOS
      #include "AudioDriver_ios.hpp"
    #elif DM_PLATFORM_LINUX
      #include "AudioDriver_alsa.hpp"
    #endif
  #endif
#endif

namespace dinahmoe {
namespace audioengine {

template <bool IS_REALTIME>
struct MutexChooser {
  typedef dinahmoe::synchro::NullMutex MUTEX_T;
};

template <>
struct MutexChooser<true> {
  typedef std::mutex MUTEX_T;
};


template <class AUDIODRIVER_T>
class AudioEngine  :
  public AUDIODRIVER_T,
  public AudioContext,
  public synchro::TimeProvider {
public:

  // factory method

  AudioEngine(DmDictionary& params);
  ~AudioEngine();
  void start();
  void stop();
  dm_time_seconds getCurrentTime();
  dm_time_seconds getLatency();
private:
  int cb(int channelsIn_, float** inPlaceInputBuffer,
    int channelsOut_, float** inPlaceOutputBuffer,
    int numberOfFrames);

  RingBuffer<float>* m_inputRingBuffers;
  RingBuffer<float>* m_outputRingBuffers;
  float** m_inputBuffer;
  float** m_outputBuffer;
  size_t m_ringBufferSize;

  #ifdef AE_PROFILE
  typedef struct AeProfilingStruct_ {
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point end;
    unsigned long long sample;
  } AeProfilingStruct;
  bool m_createProfile;
  std::chrono::system_clock::time_point m_startTime;
  std::string m_profilingFilePath;
  std::vector<AeProfilingStruct> m_profilingList;
  typename std::vector<AeProfilingStruct>::iterator profit;
  unsigned long long m_currentSampleNumber;
  #endif
};

template <class AUDIODRIVER_T>
AudioEngine<AUDIODRIVER_T>::AudioEngine(DmDictionary& params) :
  AUDIODRIVER_T(params,
  [&](int channelsIn_, float** inPlaceInputBuffer,
    int channelsOut_, float** inPlaceOutputBuffer, int nsamples) {
      return this->AudioEngine<AUDIODRIVER_T>::cb(channelsIn_, inPlaceInputBuffer, channelsOut_, inPlaceOutputBuffer, nsamples);
    }
  ),
  AudioContext(((AUDIODRIVER_T*)this)->sampleRate(), ((AUDIODRIVER_T*)this)->outputChannels(), ((AUDIODRIVER_T*)this)->bufferSize(), AUDIODRIVER_T::IS_REALTIME),
  m_ringBufferSize(0) {

  if (AUDIODRIVER_T::inputChannels()) {
    m_inputBuffer = new float*[AUDIODRIVER_T::inputChannels()];
    m_inputRingBuffers = new RingBuffer<float>[AUDIODRIVER_T::inputChannels()];
  }
  if (AUDIODRIVER_T::outputChannels()) {
    m_outputBuffer = new float*[AUDIODRIVER_T::outputChannels()];
    m_outputRingBuffers = new RingBuffer<float>[AUDIODRIVER_T::outputChannels()];
  }
  // compute a good ring buffer size
  size_t ringBuffersSize = (AUDIODRIVER_T::sampleRate()/2 > AUDIODRIVER_T::bufferSize()*3) ? AUDIODRIVER_T::sampleRate()/2 : AUDIODRIVER_T::bufferSize() * 3;

  for (int i = 0; i < AUDIODRIVER_T::inputChannels(); ++i) {
    m_inputRingBuffers[i].resize(ringBuffersSize);
    m_inputRingBuffers[i].zero(AUDIODRIVER_T::bufferSize());
    m_inputBuffer[i] = new float[AUDIODRIVER_T::sampleRate()/2];
  }
  for (int i = 0; i < AUDIODRIVER_T::outputChannels(); ++i) {
    m_outputRingBuffers[i].resize(ringBuffersSize);
    m_outputRingBuffers[i].zero(AUDIODRIVER_T::bufferSize());
    m_outputBuffer[i] = new float[AUDIODRIVER_T::sampleRate()/2];
  }

  m_ringBufferSize = AUDIODRIVER_T::bufferSize();

  #ifdef AE_PROFILE
  m_createProfile = params.keyExists("profilingFilePath");
  m_currentSampleNumber = 0;
  if (m_createProfile) {
    m_profilingFilePath = params.getStringValue("profilingFilePath");
    m_profilingList.resize(100000);
    profit = m_profilingList.begin();
  }
  m_startTime = std::chrono::system_clock::now();
  #endif
}

template <class AUDIODRIVER_T>
AudioEngine<AUDIODRIVER_T>::~AudioEngine() {
  dmaf_log(Log::Debug, "Initialization: Deleting AudioEngine");

  #ifdef AE_PROFILE
  if (m_createProfile) {
    std::ofstream osout(m_profilingFilePath);
    for (auto itl = m_profilingList.begin(); itl != profit; ++itl) {
      osout << itl->sample
        << " " << std::chrono::duration_cast<std::chrono::microseconds>(itl->start - m_startTime).count()
        << " " << std::chrono::duration_cast<std::chrono::microseconds>(itl->end - m_startTime).count()
        << " " << std::chrono::duration_cast<std::chrono::microseconds>(itl->end - itl->start).count()
        << std::endl;
    }
    osout
      << std::endl
      << std::endl
      << "MATLAB"
      << std::endl;
    osout << "TIMES = zeros(" << m_profilingList.size() << ",1);" << std::endl;
    osout << "DURATIONS = zeros(" << m_profilingList.size() << ",1);" << std::endl;
    int count = 1;
    for (auto itl = m_profilingList.begin(); itl != profit; ++itl) {
      osout << "TIMES(" << count << ") = " << ((double)itl->sample) / AudioContext::getSampleRate() << ";" << std::endl;
      osout << "DURATIONS(" << count << ") = " << std::chrono::duration_cast<std::chrono::microseconds>(itl->end - itl->start).count() << ";" << std::endl;
      ++count;
    }
    osout.close();
  }
  #endif
  for (int i = 0; i < AUDIODRIVER_T::inputChannels(); ++i) {
    delete [] m_inputBuffer[i];
  }
  for (int i = 0; i < AUDIODRIVER_T::outputChannels(); ++i) {
    delete [] m_outputBuffer[i];
  }
  if (AUDIODRIVER_T::inputChannels()) {
   delete [] m_inputBuffer;
   delete [] m_inputRingBuffers;
  }
  if (AUDIODRIVER_T::outputChannels()) {
    delete [] m_outputBuffer;
    delete [] m_outputRingBuffers;
  }

}

template <class AUDIODRIVER_T>
void AudioEngine<AUDIODRIVER_T>::start() {
  dmaf_log(Log::Debug, "Initialization: AudioEngine::startRendering");
  ((AudioContext*)this)->start();
  ((AUDIODRIVER_T*)this)->startRendering();
}

template <class AUDIODRIVER_T>
void AudioEngine<AUDIODRIVER_T>::stop() {
  ((AudioContext*)this)->stop();
  ((AUDIODRIVER_T*)this)->stopRendering();
  // clean things up
  ((AudioContext*)this)->stopAllSourceNodes();
  ((AudioContext*)this)->updateInternalState();
  dmaf_log(Log::Debug, "Initialization: AudioEngine::stopRendering");
}


template <class AUDIODRIVER_T>
int AudioEngine<AUDIODRIVER_T>::cb(int channelsIn_, float** inPlaceInputBuffer,
    int channelsOut_, float** inPlaceOutputBuffer,
    int numberOfFrames) {

  m_ringBufferSize += numberOfFrames;
  for (int i = 0; i < channelsIn_; ++i) {
    m_inputRingBuffers[i].write(inPlaceInputBuffer[i], numberOfFrames);
  }

  while (m_ringBufferSize >= AudioContext::getBufferSize()) {
    if (channelsIn_) {
      for (int i = 0; i < channelsIn_; ++i) {
        m_inputRingBuffers[i].read(m_inputBuffer[i], AudioContext::getBufferSize(), true);
      }
    }
    // notify (asynchronously) the listeners
    this->TimeProvider::Notify();
    #ifdef AE_PROFILE
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    #endif
    ((AudioContext*)this)->audioCallback(m_inputBuffer, channelsIn_, m_outputBuffer, channelsOut_, AudioContext::getBufferSize());
    if (channelsOut_) {
      for (int i = 0; i < channelsOut_; ++i) {
        m_outputRingBuffers[i].write(m_outputBuffer[i], AudioContext::getBufferSize());
      }
    }
    #ifdef AE_PROFILE
    profit->start = start;
    profit->end = std::chrono::system_clock::now();
    profit->sample = m_currentSampleNumber;
    m_currentSampleNumber += numberOfFrames;
    ++profit;
    #endif
    m_ringBufferSize -= AudioContext::getBufferSize();
  }
  if (channelsOut_) {
    for (int i = 0; i < channelsOut_; ++i) {
      m_outputRingBuffers[i].read(inPlaceOutputBuffer[i], numberOfFrames, true);
    }
  }
  return numberOfFrames;
}

template <class AUDIODRIVER_T>
dm_time_seconds AudioEngine<AUDIODRIVER_T>::getLatency()  {
  return (((AudioContext*)this)->getBufferSize() *  dm_time_seconds(AUDIODRIVER_T::IS_REALTIME? 5 : 0) / getSampleRate());
}

template <class AUDIODRIVER_T>
dm_time_seconds AudioEngine<AUDIODRIVER_T>::getCurrentTime()  {
  return ((AudioContext*)this)->getCurrentTime();
}

#if DMF_USES_OFFLINE_RENDERING
  typedef audioengine::AudioEngine<audiodriver::AudioDriver_audiofile> AudioEngineT;
#else
  #if DMF_USES_EXTERNAL_AUDIO
    typedef audioengine::AudioEngine<audiodriver::AudioDriver_external> AudioEngineT;
  #else
    #if DM_PLATFORM_OSX
      typedef audioengine::AudioEngine<audiodriver::AudioDriver_osx> AudioEngineT;
    #elif DM_PLATFORM_ANDROID
      typedef audioengine::AudioEngine<audiodriver::AudioDriver_android> AudioEngineT;
    #elif DM_PLATFORM_IOS
      typedef audioengine::AudioEngine<audiodriver::AudioDriver_ios> AudioEngineT;
    #elif DM_PLATFORM_LINUX
      typedef audioengine::AudioEngine<audiodriver::AudioDriver_alsa> AudioEngineT;
    #endif
  #endif
#endif


} // audioengine
} // dinahmoe

#endif // __AudioEngine_hpp
