// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
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

#include "simplesensorsystem.h"
#include "phase_space/PhaseSpaceSnapshot.h"

using namespace ros;

// used to update objects through a mocap system
class PhaseSpaceMocapClient : public SimpleSensorSystem
{
public:
    PhaseSpaceMocapClient(EnvironmentBase* penv) : SimpleSensorSystem(penv, "phasespace"), _bSubscribed(false)
    {
        RegisterXMLReader(GetEnv()); // just in case, register again
    }
    virtual ~PhaseSpaceMocapClient() {
        Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        if( !SimpleSensorSystem::Init(sinput) )
            return false;

        _phasespacetopic = "phase_space_snapshot";
        sinput >> _phasespacetopic;
        startsubscriptions();
        return _bSubscribed;
    }

    virtual void Destroy()
    {
        stopsubscriptions();
        SimpleSensorSystem::Destroy();
    }

    static void RegisterXMLReader(EnvironmentBase* penv)
    {
        if( penv != NULL )
            penv->RegisterXMLReader("phasespace", PhaseSpaceMocapClient::CreateMocapReader);
    }

    static BaseXMLReader* CreateMocapReader(KinBody* parent, const char **atts)
    {
        return new MocapXMLReader("phasespace", NULL, atts);
    }

private:
    node* check_roscpp()
    {
        // start roscpp
        ros::node* pnode = ros::node::instance();

        if( pnode && !pnode->checkMaster() ) {
            ros::fini();
            delete pnode;
            return NULL;
        }

        if (!pnode) {
            int argc = 0;
            char strname[256] = "nohost";
            gethostname(strname, sizeof(strname));
            strcat(strname,"_rosoct");

            ros::init(argc,NULL);
            
            pnode = new ros::node(strname, ros::node::DONT_HANDLE_SIGINT|ros::node::ANONYMOUS_NAME|ros::node::DONT_ADD_ROSOUT_APPENDER);
            
            bool bCheckMaster = pnode->checkMaster();
            ros::fini();
            delete pnode;

            if( !bCheckMaster ) {
                RAVELOG_ERRORA("ros not present");
                return NULL;
            }
        
            ros::init(argc,NULL);
            pnode = new ros::node(strname, ros::node::DONT_HANDLE_SIGINT|ros::node::ANONYMOUS_NAME);
            RAVELOG_DEBUGA("new roscpp node started");
        }

        return pnode;
    }

    void startsubscriptions()
    {
        // check if thread launched
        _bSubscribed = false;
        ros::node* pnode = check_roscpp();
        if( pnode != NULL ) {
            _bSubscribed = pnode->subscribe(_phasespacetopic, _snapshot, &PhaseSpaceMocapClient::newdatacb, this, 10);
            if( _bSubscribed )
                RAVELOG_DEBUGA("subscribed to %s\n", _phasespacetopic.c_str());
            else
                RAVELOG_ERRORA("failed to subscribe to %s\n", _phasespacetopic.c_str());
        }
    }

    void stopsubscriptions()
    {
        if( _bSubscribed ) {
            ros::node* pnode = ros::node::instance();
            if( pnode && pnode->checkMaster() ) {
                pnode->unsubscribe(_phasespacetopic.c_str());
                RAVELOG_DEBUGA("unsubscribe from %s\n", _phasespacetopic.c_str());
            }
            _bSubscribed = false;
        }
    }

    void newdatacb()
    {
        list< SNAPSHOT > listbodies;
        boost::mutex::scoped_lock lock(_mutex);
        RAVELOG_VERBOSEA("cb\n");

        for (unsigned int i=0; i<_snapshot.get_bodies_size(); i++) {
            const phase_space::PhaseSpaceBody& psbody = _snapshot.bodies[i];

            boost::shared_ptr<BODY> b;
            FOREACH(it, _mapbodies) {
                if( it->second->_initdata.id == psbody.id ) {
                    b = it->second;
                    break;
                }
            }

            if( !b || !b->IsEnabled() )
                continue;

            listbodies.push_back(SNAPSHOT(b, GetTransform(psbody.pose)));
        }

        UpdateBodies(listbodies);

        // try to add the left-over objects
    }

    Transform GetTransform(const std_msgs::Transform& pose)
    {
        return Transform(Vector(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), Vector(pose.translation.x, pose.translation.y, pose.translation.z));
    }

    phase_space::PhaseSpaceSnapshot _snapshot;
    string _phasespacetopic;
    bool _bSubscribed;
};

#endif
