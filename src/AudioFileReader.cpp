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
 
//  Created by Alessandro Saccoia on 8/27/12.

#include "AudioFileReader.hpp"
#include "Log.h"
#include "AudioFormatsManager.hpp"

#include <iostream>
#include <exception>

namespace dinahmoe {
namespace audioengine {


AudioFileReader::AudioFileReader() :
  mExitCondition(0) {
 //pthread_create(&m_decodingThread, NULL, thread_function, this);
  m_formatsManager = new assets::AudioFormatsManager(); 
}

AudioFileReader::~AudioFileReader() {
  //pthread_join(m_decodingThread, &exitStatus);
  delete m_formatsManager;
}

void AudioFileReader::threadFunction() {
  while (mExitCondition != 1) {
    if (m_decodingQueue.GetSize() == 0) {
      //sleep(50000); // 50 ms
    } else {
      DecodingJob* job;
      if ((job = m_decodingQueue.PopAll())) {
          while (job != 0) {
              job->decode();
              job = job->mpNext;
              delete job;
          }
      }
    }
  }
}

void AudioFileReader::decodeAudioFileAsync(std::string path, InMemoryBuffer* destination) {
  m_decodingQueue.Push(new DecodingJob(this, path, destination));
}

void AudioFileReader::decodeAudioFileSync(std::string path, InMemoryBuffer* destination) {
  // valgrind -v --leak-check=full ./DmafRender  2> dmafLog.txt
  #if DEBUG_VALGRIND
  destination->sampleRate = 44100.0F;
  destination->buffer.resize(2, 44100);
  return;
  #endif
  DecodingJob* job = new DecodingJob(this, path, destination);
  job->decode();
  delete job;
}

AudioFileReader::DecodingJob::DecodingJob(AudioFileReader* parent_, std::string path, InMemoryBuffer* destination) :
  mpNext(0),
  m_parent(parent_),
  m_path(path),
  m_destination(destination) {

}

void AudioFileReader::DecodingJob::decode() {
  try {
    if (m_parent->m_formatsManager->loadFile(m_path, m_destination->buffer, m_destination->sampleRate)) {
      m_destination->setIsReady();
    }
  } catch (std::runtime_error ex) {
    std::cerr << ex.what() << std::endl;
  }
}

} // DMAF
} // AudioEngine
