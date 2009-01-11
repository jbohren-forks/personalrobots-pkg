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
// \author Rosen Diankov
#ifndef COLLISIONMAP_MOCAP_SYSTEM
#define COLLISIONMAP_MOCAP_SYSTEM

#include "rossensorsystem.h"
#include <collision_map/CollisionMap.h>

// used to update objects through a mocap system
class CollisionMapXMLID
{
public:
    static const char* GetXMLId() { return "collisionmap"; }
};

class CollisionMapSystem : public ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>
{
public:
    CollisionMapSystem(EnvironmentBase* penv)
        : ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>(penv), _nNextId(1)
    {
    }

    virtual bool Init(istream& sinput)
    {
        _topic = "collision_map";
        return ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>::Init(sinput);
    }

private:
    void newdatacb()
    {
        // create the new kinbody
        GetEnv()->LockPhysics(true);
        KinBody* pbody = GetEnv()->CreateKinBody();

        _vaabbs.resize(_topicmsg.boxes.size());
        vector<AABB>::iterator itab = _vaabbs.begin();
        FOREACH(itmsgab, _topicmsg.boxes) {
            itab->pos = Vector(itmsgab->center.x, itmsgab->center.y, itmsgab->center.z);
            itab->extents = Vector(itmsgab->extents.x, itmsgab->extents.y, itmsgab->extents.z);
            //RAVELOG_VERBOSEA("pos%d: %f %f %f\n", (int)(itmsgab-_topicmsg.boxes.begin()), itab->pos.x, itab->pos.y, itab->pos.z);
            ++itab;
        }

        if( !pbody->InitFromBoxes(_vaabbs, true) ) {
            RAVELOG_ERRORA("failed to create collision map\n");
            delete pbody;
            return;
        }

        // append an id to the body
        stringstream ss;
        ss << "CollisionMap" << _nNextId++;
        pbody->SetName(ss.str().c_str());

        // add the new kinbody
        if( !GetEnv()->AddKinBody(pbody) ) {
            RAVELOG_ERRORA("failed to add body %S\n", pbody->GetName());
            delete pbody;
            return;
        }

        {
            boost::mutex::scoped_lock lock(_mutex);

            // remove all unlocked bodies
            TYPEOF(_mapbodies.begin()) itbody = _mapbodies.begin();
            while(itbody != _mapbodies.end()) {
                if( !itbody->second->IsLocked() ) {
                    BODY* b = itbody->second.get();
                    KinBody::Link* plink = itbody->second->GetOffsetLink();
                    assert( plink != NULL );
                    GetEnv()->RemoveKinBody(plink->GetParent());
                    _mapbodies.erase(itbody++);
                }
                else
                    ++itbody;
            }
        }
        
        GetEnv()->LockPhysics(false);

        MocapData* pdata = new MocapData();
        pdata->strOffsetLink = pbody->GetLinks().front()->GetName();
        BODY* b = AddKinBody(pbody, pdata);
        if( b == NULL ) {
            RAVELOG_ERRORA("removing/destroying kinbody\n");
            GetEnv()->RemoveKinBody(pbody, true);
        }
    }

    Transform GetTransform(const std_msgs::Transform& pose)
    {
        return Transform(Vector(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), Vector(pose.translation.x, pose.translation.y, pose.translation.z));
    }

    vector<AABB> _vaabbs;
    int _nNextId;
};

#endif
