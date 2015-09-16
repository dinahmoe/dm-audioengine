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
//
//  AudioListener.h
//  Nodal
//
//  Created by Thoren Horstmann on 04.09.15.
//
//

#ifndef Nodal_AudioListener_h
#define Nodal_AudioListener_h

class AudioListener{
public:
  AudioListener(float dopplerFactor = 1.F, float speedOfSound = 343.3F) :
  m_Position(0,0,0),
  m_OrientationUp(0,1,0),
  m_OrientationFront(0,0,-1),
  m_Velocity(),
  m_dopplerFactor(dopplerFactor),
  m_speedOfSound(speedOfSound){}
  void setPosition(float x, float y, float z){
    m_Position.setXYZ(x,y,z);
  }
  void setOrientation(float x, float y, float z , float xUp, float yUp, float zUp){
    m_OrientationFront.setXYZ(x,y,z);
    m_OrientationUp.setXYZ(xUp,yUp,zUp);
  }
  void setVelocity(float x, float y, float z){
    m_Velocity.setXYZ(x,y,z);
  }
  void setDopplerFactor(float df){
    m_dopplerFactor = df;
  }
  void setSpeedOfSound(float sos){
    m_speedOfSound = sos;
  }
  Vec3<float> m_Position;
  Vec3<float> m_OrientationUp;
  Vec3<float> m_OrientationFront;
  Vec3<float> m_Velocity;

private:
  float m_dopplerFactor;
  float m_speedOfSound;
};

#endif
