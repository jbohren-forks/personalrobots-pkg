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
        : ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>(penv), _robotid(0), _nNextId(1), _bEnableCollisions(true)
    {
    }
    virtual ~CollisionMapSystem() {
        stopsubscriptions(); // need to stop the subscriptions because the virtual destructor will not call the overridden stopsubscriptions
        ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>::Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        _robotid = 0;
        _topic = "collision_map";

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;

            if( stricmp(cmd.c_str(), "topic") == 0 )
                sinput >> _topic;
            else if( stricmp(cmd.c_str(), "collisions") == 0 )
                sinput >> _bEnableCollisions;
            else if( stricmp(cmd.c_str(), "robot") == 0 ) {
                int id;
                sinput >> id;
                KinBody* pbody = GetEnv()->GetBodyFromNetworkId(id);
                if( pbody != NULL )
                    _robotid = id;

                if( _robotid == 0 )
                    RAVELOG_WARNA("failed to find robot with id %d\n", id);
            }
            else break;
            
            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }

        startsubscriptions();
        return _bSubscribed;
    }

private:
    virtual void startsubscriptions()
    {
        ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>::startsubscriptions();
        
        if( _bSubscribed ) {
            boost::mutex::scoped_lock lock(_mutex);
            ros::node* pnode = check_roscpp_nocreate();
            if( pnode != NULL ) { 
                double tf_cache_time_secs;
                pnode->param("~tf_cache_time_secs", tf_cache_time_secs, 10.0);
                if (tf_cache_time_secs < 0)
                    RAVELOG_ERRORA("ROSSensorSystem: Parameter tf_cache_time_secs<0 (%f)\n", tf_cache_time_secs);
                unsigned long long tf_cache_time = tf_cache_time_secs*1000000000ULL;
                _tf.reset(new tf::TransformListener(*pnode, true, tf_cache_time));
                RAVELOG_INFOA("ROSSensorSystem: TF Cache Time: %f Seconds\n", tf_cache_time_secs);

                // **** Set TF Extrapolation Limit ****
                double tf_extrap_limit_secs ;
                pnode->param("~tf_extrap_limit", tf_extrap_limit_secs, 0.00);
                if (tf_extrap_limit_secs < 0.0)
                    RAVELOG_ERRORA("ROSSensorSystem: parameter tf_extrap_limit<0 (%f)\n", tf_extrap_limit_secs);

                ros::Duration extrap_limit;
                extrap_limit.fromSec(tf_extrap_limit_secs);
                _tf->setExtrapolationLimit(extrap_limit);
                RAVELOG_INFOA("ROSSensorSystem: tf extrapolation Limit: %f Seconds\n", tf_extrap_limit_secs);
            }
        }
    }

    virtual void stopsubscriptions()
    {
        ROSSensorSystem<collision_map::CollisionMap, CollisionMapXMLID>::stopsubscriptions();
        _tf.reset();
    }

    void newdatacb()
    {
        // create the new kinbody
        GetEnv()->LockPhysics(true);

        Transform tcollision;
        string strrobotbaselink;
        bool bHasRobotTransform = false;

        if( _robotid != 0 ) {
            KinBody* pbody = GetEnv()->GetBodyFromNetworkId(_robotid);
            if( pbody != NULL ) {
                bHasRobotTransform = true;
                tcollision = pbody->GetTransform();
                strrobotbaselink = _stdwcstombs(pbody->GetLinks().front()->GetName());
            }
        }

        if( bHasRobotTransform && !!_tf ) {
            tf::Stamped<btTransform> bttransform;

            try {
                _tf->lookupTransform(strrobotbaselink, _topicmsg.header.frame_id, _topicmsg.header.stamp, bttransform);
                tcollision = tcollision * GetTransform(bttransform);
            }
            catch(tf::TransformException& ex) {
                try {
                    _tf->lookupTransform(strrobotbaselink, _topicmsg.header.frame_id, ros::Time(), bttransform);
                    tcollision = tcollision * GetTransform(bttransform);
                }
                catch(tf::TransformException& ex) {
                    RAVELOG_WARNA("failed to get tf frames %s (robot link:%s)\n", _topicmsg.header.frame_id.c_str(), strrobotbaselink.c_str());
                    return;
                }
            }
        }

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

        XMLData* pdata = new XMLData();
        pdata->strOffsetLink = pbody->GetLinks().front()->GetName();
        BODY* b = AddKinBody(pbody, pdata);
        if( b == NULL ) {
            RAVELOG_ERRORA("removing/destroying kinbody\n");
            GetEnv()->RemoveKinBody(pbody, true);
        }
    }

    Transform GetTransform(const btTransform& bt)
    {
        btQuaternion q = bt.getRotation();
        btVector3 o = bt.getOrigin();
        return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
    }

    boost::shared_ptr<tf::TransformListener> _tf;
    int _robotid;
    vector<AABB> _vaabbs;
    int _nNextId;
    bool _bEnableCollisions;
};

#endif
