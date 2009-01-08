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
#ifndef ROS_SENSORSYSTEM_SYSTEM
#define ROS_SENSORSYSTEM_SYSTEM

#include "simplesensorsystem.h"

// used to update objects through a mocap system
template <typename T, typename XMLID>
class ROSSensorSystem : public SimpleSensorSystem<XMLID>
{
public:
    ROSSensorSystem(EnvironmentBase* penv) : SimpleSensorSystem<XMLID>(penv), _bSubscribed(false)
    {
        
    }
    virtual ~ROSSensorSystem() {
        Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;

            if( stricmp(cmd.c_str(), "topic") == 0 )
                sinput >> _topic;
            else break;

            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }

        startsubscriptions();
        return _bSubscribed;
    }

    virtual void Destroy()
    {
        stopsubscriptions();
        SimpleSensorSystem<XMLID>::Destroy();
    }

protected:
    virtual void startsubscriptions()
    {
        // check if thread launched
        _bSubscribed = false;
        ros::node* pnode = check_roscpp();
        if( pnode != NULL ) {
            _bSubscribed = pnode->subscribe(_topic, _topicmsg, &ROSSensorSystem::newdatacb, this, 10);
            if( _bSubscribed )
                RAVELOG_DEBUGA("subscribed to %s\n", _topic.c_str());
            else
                RAVELOG_ERRORA("failed to subscribe to %s\n", _topic.c_str());
        }
    }

    virtual void stopsubscriptions()
    {
        if( _bSubscribed ) {
            ros::node* pnode = check_roscpp_nocreate();
            if( pnode != NULL ) {
                pnode->unsubscribe(_topic.c_str());
                RAVELOG_DEBUGA("unsubscribe from %s\n", _topic.c_str());
            }
            _bSubscribed = false;
        }
    }

    virtual void newdatacb()
    {
    }

    T _topicmsg;
    string _topic;
    bool _bSubscribed;
};

#endif
