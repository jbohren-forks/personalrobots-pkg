
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Eric Berger
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

//The robot model is populated by the control code infrastructure and used by all the controllers to read mechanism state and command mechanism motion.

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "mechanism_model/link.h"
#include "mechanism_model/joint.h"
#include "mechanism_model/transmission.h"

namespace mechanism {
  class Robot{
  public:
    Robot(char *ns){}
    ~Robot()
    {
      std::vector<Transmission *>::size_type t;
      for (t = 0; t < transmissions_.size(); ++t)
        delete transmissions_[t];
      std::vector<Joint *>::size_type j;
      for (j = 0; j < joints_.size(); ++j)
        delete joints_[j];
    }

    char *name;
    Link *link;
    int numLinks;
    Joint *joint;  // TODO: delete
    int numJoints;
    int numTransmissions;
    SimpleTransmission *transmission;  // TODO: delete

    std::vector<Joint*> joints_;
    std::vector<Transmission*> transmissions_;
  };
}

#endif
