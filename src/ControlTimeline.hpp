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
 * \file ControlTimeline.hpp
 *
 * \author Alessandro Saccoia
 * \date 9/27/12
 *
 * \todo dezipper values
 *
 */

#include <list>
#include <mutex>
#include "MemoryPool.hpp"
#include "DmTypedefs.hpp"
#include "AudioBuffer.hpp"

#ifndef AudioEngine_ControlTimeline_hpp
#define AudioEngine_ControlTimeline_hpp

namespace dinahmoe {
namespace audioengine {

class AudioNodeInput;

/*! \brief Represents an event scheduled for the timeline */
class TimelineEvent {
 public:
  enum Type {
      SetValue,
      LinearRampToValue,
      ExponentialRampToValue,
      SetTargetValue,
      SetValueCurve,
      CancelScheduledValues,
      LastType
  };
  /*! \brief Default constructor
  */
  TimelineEvent() : m_hasBeenSeen(false) { }

  /*! \brief Constructor
   *
   * Also sets the initial value of the control timeline
   */
  TimelineEvent(Type type, float value, dm_samples_t time_, float timeConstant, dm_time_seconds duration);

  unsigned type() const { return m_type; }
  float value() const { return m_value; }
  float timeConstant() const { return m_timeConstant; }
  dm_samples_t duration() const { return m_duration; }
  
  inline bool operator< (const TimelineEvent& rhs) const {
    return this->time < rhs.time;
  }
  
  inline bool operator== (const TimelineEvent& rhs) const {
    return this->time == rhs.time;
  }

  inline const TimelineEvent& operator=(const TimelineEvent& rhs) {
    time = rhs.time;
    m_type = rhs.m_type;
    m_timeConstant = rhs.m_type;
    m_duration = rhs.m_duration;
    m_value = rhs.m_value;
    m_slope = rhs.m_slope;
    m_hasBeenSeen = rhs.m_hasBeenSeen;
    return *this;
  }
  
  
  dm_samples_t time;

  unsigned m_type;
  float m_value;
  float m_timeConstant;
  dm_samples_t m_duration;
  float m_slope;
  bool m_hasBeenSeen;
};

/*! \brief Handles value transformations for AudioNode in Control modality
 *  
 */
class ControlTimeline {
friend class AudioNodeInput;
public:
  /*! \brief Constructor
   *
   * Also sets the initial value of the control timeline
   */
  ControlTimeline(AudioNodeInput* input_, float initialValue_);
  ~ControlTimeline();
  
  // These methods are called from the main thread: they all access the memory pool
  // that is kept by the context in order to create an event. At this point they lock()
  // on m_newEventsList, push back the new item, sort m_newEventsList and unlock
  void setValueAtTime(float value, dm_time_seconds time);
  void linearRampToValueAtTime(float value, dm_time_seconds time);
  void exponentialRampToValueAtTime(float value, dm_time_seconds time);
  void cancelScheduledValues(dm_time_seconds startTime);
  
  /*! \brief Called from the AudioNodeInput to optimize in case of steady state
   *
   * This is called at the beginning of each rendering quantum to see if
   * the value has to be considered constant and optimize accordingly
   */
  bool hasEvents(size_t numFrames);
  
  float getLastValue() const {
    return m_lastValue;
  }

  
  struct EqualTimeFunctor {
    dm_time_seconds time;
    EqualTimeFunctor(dm_time_seconds& time_) : time(time_) {}
    bool operator()(TimelineEvent const& event) {
      return event.time == time;
    }
  };

  
  /*! \brief Fills the buffer_ parameter with an array of samples */
  AudioBuffer& pull(AudioBuffer& buffer_, size_t numFrames);
  
private:
  /*! \brief A reference to the input node, mostly to have access to the context */
  AudioNodeInput* m_nodeInput;
  
  /*! \brief A list of present or future TimelineEvents used by the processing thread  */
  std::list<TimelineEvent*> m_eventList;
  /*! \brief List of TimelineEvents filled from the main thread  */
  std::list<TimelineEvent*> m_newEventsList;
  
  float m_lastValue;
  
  /*! \brief Mutex   */
  std::mutex m_eventsLock;
};

}
}

#endif