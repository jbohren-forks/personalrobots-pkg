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
#ifndef OBJECTTRANSFORM_SENSOR_SYSTEM
#define OBJECTTRANSFORM_SENSOR_SYSTEM

#include <tf/transform_listener.h>
#include <checkerboard_detector/ObjectDetection.h>

#include "rossensorsystem.h"

// used to update objects through a mocap system
class ObjectTransformXMLID
{
public:
    static const char* GetXMLId() { return "objecttransform"; }
};

class ObjectTransformSystem : public ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>
{
public:
    ObjectTransformSystem(EnvironmentBase* penv)
        : ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>(penv), _probot(NULL), nNextId(1)
    {
    }

    virtual bool Init(istream& sinput)
    {
        _probot = NULL;
        _topic = "ObjectDetection";
        _fThreshSqr = 0.05*0.05f;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;

            if( stricmp(cmd.c_str(), "topic") == 0 )
                sinput >> _topic;
            else if( stricmp(cmd.c_str(), "thresh") == 0 )
                sinput >> _fThreshSqr;
            else if( stricmp(cmd.c_str(), "robot") == 0 ) {
                int id;
                sinput >> id;
                KinBody* pbody = GetEnv()->GetBodyFromNetworkId(id);
                if( pbody->IsRobot() )
                    _probot = (RobotBase*)pbody;

                if( _probot == NULL )
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
        ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>::startsubscriptions();
        
        if( _bSubscribed ) {
            ros::node* pnode = check_roscpp_nocreate();
            if( pnode != NULL ) { 
                double tf_cache_time_secs;
                pnode->param("~tf_cache_time_secs", tf_cache_time_secs, 10.0);
                if (tf_cache_time_secs < 0)
                    RAVELOG_ERRORA("ROSSensorSystem: Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs);
                unsigned long long tf_cache_time = tf_cache_time_secs*1000000000ULL;
                _tf.reset(new tf::TransformListener(*pnode, true, tf_cache_time));
                RAVELOG_INFOA("ROSSensorSystem: TF Cache Time: %f Seconds", tf_cache_time_secs);

                // **** Set TF Extrapolation Limit ****
                double tf_extrap_limit_secs ;
                pnode->param("~tf_extrap_limit", tf_extrap_limit_secs, 0.00);
                if (tf_extrap_limit_secs < 0.0)
                    RAVELOG_ERRORA("ROSSensorSystem: parameter tf_extrap_limit<0 (%f)", tf_extrap_limit_secs);

                ros::Duration extrap_limit;
                extrap_limit.fromSec(tf_extrap_limit_secs);
                _tf->setExtrapolationLimit(extrap_limit);
                RAVELOG_INFOA("ROSSensorSystem: tf extrapolation Limit: %f Seconds", tf_extrap_limit_secs);
            }
        }
    }

    virtual void stopsubscriptions()
    {
        ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>::stopsubscriptions();
        _tf.reset();
    }

    void newdatacb()
    {
        list< SNAPSHOT > listbodies;
        list< const checkerboard_detector::Object6DPose* > listnewobjs;

        {
            boost::mutex::scoped_lock lock(_mutex);
            TYPEOF(_mapbodies) mapbodies = _mapbodies;
            std_msgs::PoseStamped posestamped, poseout;
            Transform trobot;

            if( _probot != NULL && _topicmsg.objects.size() > 0 ) {
                GetEnv()->LockPhysics(true);
                trobot = _probot->GetTransform();
                GetEnv()->LockPhysics(false);
            }

            FOREACHC(itobj, _topicmsg.objects) {
                boost::shared_ptr<BODY> b;
                
                Transform tnew;

                // if on robot, have to find the global transformation
                if( _probot != NULL ) {
                    posestamped.pose = itobj->pose;
                    posestamped.header = _topicmsg.header;
                    
                    try {
                        _tf->transformPose(_stdwcstombs(_probot->GetLinks().front()->GetName()), posestamped, poseout);
                        tnew = trobot * _probot->GetTransform() * GetTransform(poseout.pose);
                    }
                    catch(tf::TransformException& ex) {
                        RAVELOG_WARNA("failed to get tf frames %S for object %s\n",posestamped.header.frame_id.c_str(), _probot->GetLinks().front()->GetName(), b->_initdata->sid.c_str());
                        tnew = GetTransform(itobj->pose);
                    }
                }
                else
                    tnew = GetTransform(itobj->pose);

                FOREACH(it, mapbodies) {
                    if( it->second->_initdata->sid == itobj->type ) {
                            
                        // same type matched, need to check proximity
                        Transform tbody = it->second->GetOffsetLink()->GetParent()->GetTransform();
                        if( (tbody.trans-tnew.trans).lengthsqr3() > _fThreshSqr )
                            break;

                        b = it->second;
                        mapbodies.erase(it);
                        break;
                    }
                }

                if( !b ) {
                    listnewobjs.push_back(&(*itobj));
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
        if( listnewobjs.size() > 0 ) {
            GetEnv()->LockPhysics(true);
            FOREACH(itobj, listnewobjs) {

                KinBody* pbody = GetEnv()->CreateKinBody();

                if( !pbody->Init( (*itobj)->type.c_str(), NULL ) ) {
                    RAVELOG_ERRORA("failed to create object %s\n", (*itobj)->type.c_str());
                    delete pbody;
                    continue;
                }

                // append an id to the body
                wstringstream ss;
                ss << pbody->GetName() << nNextId++;
                pbody->SetName(ss.str().c_str());

                if( !GetEnv()->AddKinBody(pbody) ) {
                    RAVELOG_ERRORA("failed to add body %S\n", pbody->GetName());
                    delete pbody;
                    continue;
                }

                if( AddKinBody(pbody, NULL) == NULL ) {
                    delete pbody;
                    continue;
                }

                pbody->SetTransform(GetTransform((*itobj)->pose));
            }
            GetEnv()->LockPhysics(false);
        }
    }

    Transform GetTransform(const std_msgs::Pose& pose)
    {
        return Transform(Vector(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z), Vector(pose.position.x, pose.position.y, pose.position.z));
    }
    Transform GetTransform(const btTransform& bt)
    {
        btQuaternion q = bt.getRotation();
        btVector3 o = bt.getOrigin();
        return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
    }

    boost::shared_ptr<tf::TransformListener> _tf;
    RobotBase* _probot; ///< system is attached to this robot
    dReal _fThreshSqr;
    int nNextId;
};

#endif

