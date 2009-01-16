// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
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
//
// author: Rosen Diankov
#ifndef PHASESPACE_MOCAP_SYSTEM
#define PHASESPACE_MOCAP_SYSTEM

#include "rossensorsystem.h"
#include "robot_msgs/MocapSnapshot.h"

// used to update objects through a mocap system
class MocapXMLID
{
public:
    static const char* GetXMLId() { return "phasespace"; }
};

class ROSMocapSystem : public ROSSensorSystem<robot_msgs::MocapSnapshot, MocapXMLID>
{
public:
    ROSMocapSystem(EnvironmentBase* penv)
        : ROSSensorSystem<robot_msgs::MocapSnapshot, MocapXMLID>(penv)
    {
    }

    virtual bool Init(istream& sinput)
    {
        _topic = "robot_msgs_snapshot";
        return ROSSensorSystem<robot_msgs::MocapSnapshot, MocapXMLID>::Init(sinput);
    }

private:
    void newdatacb()
    {
        list< SNAPSHOT > listbodies;
        list< const robot_msgs::MocapBody* > listnewbodies;

        {
            boost::mutex::scoped_lock lock(_mutex);

            for (unsigned int i=0; i<_topicmsg.get_bodies_size(); i++) {
                const robot_msgs::MocapBody& psbody = _topicmsg.bodies[i];

                boost::shared_ptr<BODY> b;
                Transform tnew = GetTransform(psbody.pose);

                FOREACH(it, _mapbodies) {
                    if( it->second->_initdata->id == psbody.id ) {
                        b = it->second;
                        break;
                    }
                }

                if( !b ) {
                    listnewbodies.push_back(&psbody);
                }
                else {
                    if( !b->IsEnabled() )
                        continue;
                    
                    listbodies.push_back(SNAPSHOT(b, tnew));
                }
            }

            UpdateBodies(listbodies);
        }

        // try to add the left-over objects
    }

    Transform GetTransform(const std_msgs::Transform& pose)
    {
        return Transform(Vector(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), Vector(pose.translation.x, pose.translation.y, pose.translation.z));
    }
};

#endif
