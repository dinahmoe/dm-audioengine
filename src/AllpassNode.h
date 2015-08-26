//
//  AllpassNode.h
//  Nodal
//
//  Created by Thoren Horstmann on 26.08.15.
//
//

#ifndef __Nodal__AllpassNode__
#define __Nodal__AllpassNode__
//
//  PlainReverbNode.h
//  Nodal
//
//  Created by Thoren Horstmann on 26.08.15.
//
//

#ifndef __Nodal__PlainReverbNode__
#define __Nodal__PlainReverbNode__

#include "AudioNode.hpp"
#include "AudioGainNode.hpp"
#include "DelayNode.hpp"
#include "AudioContext.hpp"

namespace dinahmoe {
  namespace audioengine {
    
    class AllpassNode : public AudioNode {
    public:
      AllpassNode(AudioContext* context,float delay,float gain);
      ~AllpassNode();
      inline const char* getType() { return "AllpassNode"; }
      void processInternal(int numSamples, int outputRequesting);
    private:
      RefCounted<AudioGainNode>* m_gains;
      RefCounted<DelayNode> m_delay;
      RefCounted<SummingNode>* m_sums;
    };
    
  } // AUDIOENGINE
} // DMAF

#endif /* defined(__Nodal__PlainReverbNode__) */

#endif /* defined(__Nodal__AllpassNode__) */
