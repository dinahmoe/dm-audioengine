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
 
//  Created by Alessandro Saccoia on 9/27/12.


/*
  example code for the reverse iterator behavior
  
  
int main(int argc, const char * argv[])
{
  int myInts[] = {2,4,2,1,1,1,1,2};
  std::list<int> listOfInts(myInts, myInts + sizeof(myInts)/sizeof(int));
  
  std::copy(listOfInts.begin(),
          listOfInts.end(),
          std::ostream_iterator<int>(std::cout, ","));
  
  std::list<int>::reverse_iterator rit = listOfInts.rbegin();
  while (rit != listOfInts.rend()) {
    std::list<int>::reverse_iterator rit2 = rit;
    std::advance(rit2, 1);
    std::cout << "Processing " << *rit << std::endl;
    while (rit2 != listOfInts.rend()) {
      if (*rit == *rit2) {
        std::cout << "found duplicate " << *rit2 << std::endl;
        listOfInts.erase(std::next(rit2).base());
      } else {
        ++rit2;
      }
    }
    std::cout << std::endl;
    ++rit;
  }
  
  std::copy(listOfInts.begin(),
          listOfInts.end(),
          std::ostream_iterator<int>(std::cout, ","));
  
  return 0;
}


Instead for the timelines:
This mess it's actually better understood with this demo program
#include <iostream>
#include <list>
#include <algorithm>
#include <cassert>

struct MyPair {
  MyPair(int t, int a, float b) : m_type(t), time(a), m_value(b) {}
 int type() { return m_type; }
 int value() { return m_value; }
 int m_type;
 int time;
 float m_value;
};


std::list<MyPair*> m_eventList;
float buffer[20];

struct prova {
  float m_lastValue;
  void compute(float* buf, int startSample, int numSamples) {
    long currentSampleInBuffer = 0;
    long endTime = startSample + numSamples - 1;
    long lastSample = startSample - 1;
    while (m_eventList.size() && ((*m_eventList.begin())->time <= endTime)) {
      auto bel = m_eventList.begin();
      int samplesToGo = (int)(*bel)->time - (int)lastSample;
      assert(samplesToGo);
      if ((*bel)->type() == 0) {
        std::fill(buf + currentSampleInBuffer, buf + currentSampleInBuffer + samplesToGo, m_lastValue);
      } else if ((*bel)->type() == 1) {
        float slope = ((*bel)->value() - m_lastValue) / ((float)(int)(*bel)->time - (int)lastSample);
        for (int i = 0; i < samplesToGo; ++i) {
          m_lastValue += slope;
          buf[currentSampleInBuffer + i] = m_lastValue;
        }
      } else {
        assert(false);
      }
      currentSampleInBuffer += samplesToGo;
      lastSample += samplesToGo;
      m_eventList.erase(m_eventList.begin());
    }
    int samplesToGo = (int)endTime - (int)lastSample;
    if (currentSampleInBuffer < (numSamples) - 1) {
      if (m_eventList.size() && ((*m_eventList.begin())->type() == 1)) {
        auto bel = m_eventList.begin();
        float slope = ((*bel)->value() - m_lastValue) / ((float)((int)(*bel)->time - (int)lastSample));
        for (int i = 0; i < samplesToGo; ++i) {
          m_lastValue += slope;
          buf[currentSampleInBuffer + i] = m_lastValue;
        }
      } else {
        std::fill(buf + currentSampleInBuffer, buf + samplesToGo, m_lastValue);
      }
    }
  }


};

int main(int argc, const char * argv[])
{
  std::fill(buffer + 0, buffer + 10, 99);
  prova provaInstance;
  provaInstance.m_lastValue = -1.0;
  m_eventList.emplace_back(new MyPair(1, 1, 1.0F));
  m_eventList.emplace_back(new MyPair(1, 2, 2.0F));
  m_eventList.emplace_back(new MyPair(0, 4, 2.0F));
  m_eventList.emplace_back(new MyPair(1, 5, 1.0F));
  m_eventList.emplace_back(new MyPair(1, 9, 2.0F));
  m_eventList.emplace_back(new MyPair(1, 11, 0.0F));
  m_eventList.emplace_back(new MyPair(1, 20, 10.0F));
  for (int i = 0; i < 100; i += 10) {
    provaInstance.compute(buffer + i, i,i+10);
  }
  for (int i = 0; i < 100; ++i) {
    std::cout << "buffer[" << i << "] = " << buffer[i] << std::endl;
  }
  return 0;
}

*/

#include "ControlTimeline.hpp"
#include "AudioContext.hpp"
#include "AudioNode.hpp"
#include "AudioNodeInput.hpp"
#include "StdAlgorithms.hpp"
#include "Log.h"

namespace dinahmoe {
namespace audioengine {

ControlTimeline::ControlTimeline(AudioNodeInput* input_, float initialValue_) :
  m_nodeInput(input_),
  m_lastValue(initialValue_) {
}
ControlTimeline::~ControlTimeline() {

}

// this is called at the beginning of each rendering quantum to see if
// the value has to be considered constant and optimize accordingly
bool ControlTimeline::hasEvents(size_t numFrames) {
  AudioContext* ctx = m_nodeInput->node()->context();
  dm_samples_t startTime = m_nodeInput->node()->context()->getCurrentSampleFrames();
  dm_samples_t endTime = startTime + numFrames;
  
  // 1. merge m_newEventsList into m_eventList
  // in the lock cannot be acquired, the new events will stay in the list for as long as we can enter it.
  // we will find in any case the events pushed back in the correct order
  if (m_eventsLock.try_lock()) {
    // we consume all the new events in various ways, and always through begin(). makes sense to iterate till size != 0
    while (m_newEventsList.size()) {
      auto pel = m_newEventsList.begin();
      auto pel_1 = std::next(m_newEventsList.begin());
      // delete from m_eventList all the events that have a time >= pel->time, and put them in the m_eventListToDelete
      if ((*pel)->type() == TimelineEvent::CancelScheduledValues) {
        std::list<TimelineEvent*>::iterator tdl = m_eventList.begin();
        while (tdl != m_eventList.end()) {
          if ((*tdl)->time >= (*pel)->time) {
            break;
          } else {
            ++tdl;
          }
        }
        if (tdl != m_eventList.end()) {
          ctx->getControlEventsToDeleteList().splice(ctx->getControlEventsToDeleteList().end(),
              m_eventList,
              tdl,
              m_eventList.end());
        }
        // delete also the event from the m_newEventsList
        ctx->getControlEventsToDeleteList().splice(ctx->getControlEventsToDeleteList().end(), m_newEventsList, pel, pel_1);
      } else {
        // otherwise just splice this event into m_eventList
        m_eventList.splice(m_eventList.end(), m_newEventsList, pel, pel_1);
      }
    }
    assert(m_newEventsList.size() == 0);
    m_eventsLock.unlock();
  }
  
  // if we have no events by now, we can be sure that the timeline is not sample accurate
  if (m_eventList.size() == 0) return false;
  
  // events at this point have been pushed_back in the order they have been added from the user code
  // we can remove duplicates by iterating backwards
  if (m_eventList.size() > 1) {
    std::list<TimelineEvent*>::reverse_iterator rit = m_eventList.rbegin();
    while (rit != m_eventList.rend()) {
      std::list<TimelineEvent*>::reverse_iterator rit2 = rit;
      std::advance(rit2, 1);
      while (rit2 != m_eventList.rend()) {
        if (*(*rit) == *(*rit2)) {
          ctx->getControlEventsToDeleteList().splice(ctx->getControlEventsToDeleteList().end(), m_eventList, std::next(rit2).base());
        } else {
          ++rit2;
        }
      }
      ++rit;
    }
  }

  // now sort the list: events could have been added with scattered times
  m_eventList.sort([](const TimelineEvent* lhs, const TimelineEvent* rhs) { return *lhs < *rhs;});
  
  // treat past events:
  // set the current value to the one of the events that had been added in the past and never processed
  while (m_eventList.size() && (*m_eventList.begin())->time < startTime) {
//    dm_debug_log(m_nodeInput->node()->context()->getLog(), Log::Debug, "Event at sample %i for time %i", startTime, (*m_eventList.begin())->time);
    auto bel = m_eventList.begin(), bel_1 = m_eventList.begin();
    ++bel_1;
    m_lastValue = (*bel)->value();
    ctx->getControlEventsToDeleteList().splice(ctx->getControlEventsToDeleteList().end(), m_eventList, bel, bel_1);
  }
  
  // still, we can be sure that the timeline is not sample accurate
  if (m_eventList.size() == 0) return false;
  
  // if we are here we have events in this frame or in the future.
  auto bel = m_eventList.begin();
  /* Line 1: if we have to ramp, we will be ramping now */
  /* Line 2: if we have a setvalue, it's important just if it happen half frame */
  return (
            ((*bel)->type() > TimelineEvent::SetValue) ||
            (((*bel)->type() == TimelineEvent::SetValue) && ((*bel)->time < endTime))
         );
  
}

// this method is triggered just if there is actually sample accurate processing
AudioBuffer& ControlTimeline::pull(AudioBuffer& buffer_, size_t numSamples) {
  assert(m_eventList.size());
  
  // OPTIMIZE make sure these calls are inlined
  AudioContext* ctx = m_nodeInput->node()->context();
  dm_samples_t startSample = ctx->getCurrentSampleFrames();
  
  dm_samples_t currentSampleInBuffer = 0;
  dm_samples_t endTime = startSample + numSamples - 1;
  dm_samples_t lastSample = startSample - 1;
  while (m_eventList.size() && ((*m_eventList.begin())->time <= endTime)) {
    auto bel = m_eventList.begin();
//    dm_debug_log(ctx->getLog(), Log::Debug, "Processing event type %i, value %f at time %i for time %i: current value %f", (*bel)->type(), (*bel)->value(), ctx->getCurrentSampleFrames(), (*bel)->time, m_lastValue);
    auto bel_1 = std::next(m_eventList.begin());
    int samplesToGo = (int)(*bel)->time - (int)lastSample;
    assert(samplesToGo);
    if ((*bel)->type() == TimelineEvent::SetValue) {
      buffer_.fill(m_lastValue, samplesToGo, currentSampleInBuffer);
      m_lastValue = (*bel)->value();
//      std::fill(buf + currentSampleInBuffer, buf + currentSampleInBuffer + samplesToGo, m_lastValue);
    } else if ((*bel)->type() == TimelineEvent::LinearRampToValue) {
      if (!(*bel)->m_hasBeenSeen) {
        (*bel)->m_slope = ((*bel)->value() - m_lastValue) / ((float)(int)(*bel)->time - (int)lastSample);
        (*bel)->m_hasBeenSeen = true;
      }
      for (int i = 0; i < samplesToGo; ++i) {
        m_lastValue += (*bel)->m_slope;
        for (unsigned int chan = 0; chan < buffer_.usedChannels; ++chan) {
          buffer_.data[chan][currentSampleInBuffer + i] = m_lastValue;
        }
      }
    } else {
      assert(false);
    }
    currentSampleInBuffer += samplesToGo;
    lastSample += samplesToGo;
    ctx->getControlEventsToDeleteList().splice(ctx->getControlEventsToDeleteList().end(),
            m_eventList, m_eventList.begin(), bel_1);
  }
  dm_samples_t samplesToGo = (int)endTime - (int)lastSample;
  if (samplesToGo > 0) {
    if (m_eventList.size() && ((*m_eventList.begin())->type() == 1)) {
      auto bel = m_eventList.begin();
      if (!(*bel)->m_hasBeenSeen) {
        (*bel)->m_slope = ((*bel)->value() - m_lastValue) / ((float)(int)(*bel)->time - (int)lastSample);
        (*bel)->m_hasBeenSeen = true;
      }
      for (unsigned int i = 0; i < samplesToGo; ++i) {
        m_lastValue += (*bel)->m_slope;
        for (unsigned int chan = 0; chan < buffer_.usedChannels; ++chan) {
          buffer_.data[chan][currentSampleInBuffer + i] = m_lastValue;
        }
      }
    } else {
      buffer_.fill(m_lastValue, samplesToGo, currentSampleInBuffer);
//      std::fill(buf + currentSampleInBuffer, buf + (numSamples - currentSampleInBuffer - 1), m_lastValue);
    }
  }

  return buffer_;
}


TimelineEvent::TimelineEvent(Type type, float value, dm_samples_t time_, float timeConstant, float duration)
    : time(time_)
    , m_type(type)
    , m_value(value)
    , m_timeConstant(timeConstant)
    , m_duration(duration)
{
//  const char* strType;
//  switch (type) {
//    case SetValue:
//      strType = "SetValue";
//      break;
//    case LinearRampToValue:
//      strType = "LinearRampToValue";
//      break;
//    case ExponentialRampToValue:
//      strType = "ExponentialRampToValue";
//      break;
//    case SetValueCurve:
//      strType = "SetValueCurve";
//      break;
//    case CancelScheduledValues:
//      strType = "CancelScheduledValues";
//      break;
//    default:
//      strType = "Unknown";
//      break;
//  };
//  time = static_cast<samples_t>(round(time_ * ENGINE->context()->getSampleRate()));
//  m_duration = static_cast<samples_t>(round(duration * ENGINE->context()->getSampleRate()));
//  dmaf_log(Log::Debug, "Scheduling: timeline event type %s at time %f for time %f (%i) duration %f",
//    strType,
//    ENGINE->getCurrentTime(),
//    time_,
//    time,
//    duration
//    );
//  
}

void ControlTimeline::setValueAtTime(float value_, dm_time_seconds time_)
{
  AudioContext* ctx = m_nodeInput->node()->context();
  TimelineEvent* pev = ctx->createTimelineEvent();
  pev->m_type = TimelineEvent::SetValue;
  pev->m_value = value_;
  pev->time = time_ * static_cast<dm_samples_t>(round(ctx->getSampleRate()));;
  pev->m_duration = .0F;
  m_eventsLock.lock();
  m_newEventsList.push_back(pev);
  m_eventsLock.unlock();
//  dm_debug_log(ctx->getLog(), Log::Debug, "Event setValue value %f at time %i for time %i", pev->m_value, ctx->getCurrentSampleFrames(), pev->time);
}

void ControlTimeline::linearRampToValueAtTime(float value, dm_time_seconds time_)
{
  AudioContext* ctx = m_nodeInput->node()->context();
  TimelineEvent* pev = ctx->createTimelineEvent();
  pev->m_type = TimelineEvent::LinearRampToValue;
  pev->m_value = value;
  pev->time = time_ * static_cast<dm_samples_t>(round(ctx->getSampleRate()));;
  pev->m_duration = .0F;
  pev->m_hasBeenSeen = false;
  m_eventsLock.lock();
  m_newEventsList.push_back(pev);
  m_eventsLock.unlock();
//  dm_debug_log(ctx->getLog(), Log::Debug, "Event linearRamp value %f at time %i for time %i", pev->m_value, ctx->getCurrentSampleFrames(), pev->time);
}

void ControlTimeline::exponentialRampToValueAtTime(float value, dm_time_seconds time_)
{
  AudioContext* ctx = m_nodeInput->node()->context();
  TimelineEvent* pev = ctx->createTimelineEvent();
  pev->m_type = TimelineEvent::ExponentialRampToValue;
  pev->m_value = value;
  pev->time = time_ * static_cast<dm_samples_t>(round(ctx->getSampleRate()));;
  pev->m_duration = .0F;
  m_eventsLock.lock();
  m_newEventsList.push_back(pev);
  m_eventsLock.unlock();
}

void ControlTimeline::cancelScheduledValues(dm_time_seconds cancellationTime_) {
  AudioContext* ctx = m_nodeInput->node()->context();
  TimelineEvent* pev = ctx->createTimelineEvent();
  pev->m_type = TimelineEvent::CancelScheduledValues;
  pev->m_value = .0F;
  pev->time = cancellationTime_ * static_cast<dm_samples_t>(round(ctx->getSampleRate()));;
  pev->m_duration = .0F;
  m_eventsLock.lock();
  m_newEventsList.push_back(pev);
  m_eventsLock.unlock();
//  dm_debug_log(ctx->getLog(), Log::Debug, "Event cancel value %f at time %i for time %i", pev->m_value, ctx->getCurrentSampleFrames(), pev->time);
}


}
}