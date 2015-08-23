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
 
//  Created by Alessandro Saccoia on 2/25/14.
/*
The ChannelSplitterNode is for use in more advanced applications and would often be used in conjunction with ChannelMergerNode.

    numberOfInputs  : 1
    numberOfOutputs : Variable N (defaults to 6) // number of "active" (non-silent) outputs is determined by number of channels in the input

    channelCountMode = "max";
    channelInterpretation = "speakers";
This interface represents an AudioNode for accessing the individual channels of an audio stream in the routing graph. It has a single input, and a number of "active" outputs which equals the number of channels in the input audio stream. For example, if a stereo input is connected to an ChannelSplitterNode then the number of active outputs will be two (one from the left channel and one from the right). There are always a total number of N outputs (determined by the numberOfOutputs parameter to the AudioContext method createChannelSplitter()), The default number is 6 if this value is not provided. Any outputs which are not "active" will output silence and would typically not be connected to anything.

*/

#ifndef __Dmaf_Offline_Audio_Renderer__ChannelSplitterNode__
#define __Dmaf_Offline_Audio_Renderer__ChannelSplitterNode__


#include "AudioNode.hpp"

namespace dinahmoe {
namespace audioengine {

class AudioParam;

class ChannelSplitterNode :
	public AudioNode {
public:	
	ChannelSplitterNode(AudioContext* context, size_t outputChannels_ = 2);
	~ChannelSplitterNode();
	void processInternal(int numSamples, int outputRequesting);
  inline const char* getType() { return "ChannelSplitterNode"; }
private:
  size_t m_outputChannels;
};


}}

#endif /* defined(__Dmaf_Offline_Audio_Renderer__ChannelSplitterNode__) */
