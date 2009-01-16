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
        : ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>(penv), _robotid(0), _nNextId(1)
    {
    }
    virtual ~ObjectTransformSystem() {
        stopsubscriptions(); // need to stop the subscriptions because the virtual destructor will not call the overridden stopsubscriptions
        ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>::Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        _robotid = 0;
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
                if( pbody != NULL )
                    _robotid = id;

                if( _robotid == 0 )
                    RAVELOG_WARNA("failed to find robot with id %d\n", id);
            }
            else if( stricmp(cmd.c_str(), "matrixoffset") == 0 ) {
                TransformMatrix tmat;
                sinput >> tmat;
                _toffset = tmat;
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
            boost::mutex::scoped_lock lock(_mutex);
            ros::Node* pnode = check_roscpp_nocreate();
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
        ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>::stopsubscriptions();
        _tf.reset();
    }

    void newdatacb()
    {
        list< SNAPSHOT > listbodies;
        list< pair<string,Transform> > listnewobjs;

        {
            boost::mutex::scoped_lock lock(_mutex);
            TYPEOF(_mapbodies) mapbodies = _mapbodies;
            std_msgs::PoseStamped posestamped, poseout;
            Transform trobot;
            string strrobotbaselink;
            bool bHasRobotTransform = false;

            if( _robotid != 0 && _topicmsg.objects.size() > 0 ) {
                GetEnv()->LockPhysics(true);
                KinBody* pbody = GetEnv()->GetBodyFromNetworkId(_robotid);
                if( pbody != NULL ) {
                    bHasRobotTransform = true;
                    trobot = pbody->GetTransform();
                    strrobotbaselink = _stdwcstombs(pbody->GetLinks().front()->GetName());
                }
                GetEnv()->LockPhysics(false);
            }

            FOREACHC(itobj, _topicmsg.objects) {
                boost::shared_ptr<BODY> b;
                
                Transform tnew;

                // if on robot, have to find the global transformation
                if( bHasRobotTransform && !!_tf ) {
                    posestamped.pose = GetPose(_toffset * GetTransform(itobj->pose));
                    posestamped.header = _topicmsg.header;
                    
                    try {
                        _tf->transformPose(strrobotbaselink, posestamped, poseout);
                        tnew = trobot * GetTransform(poseout.pose);
                    }
                    catch(tf::TransformException& ex) {

                        try {
                            // try getting the latest value by passing a 0 timestamp
                            posestamped.header.stamp = ros::Time();
                            _tf->transformPose(strrobotbaselink, posestamped, poseout);
                            tnew = trobot * GetTransform(poseout.pose);
                        }
                        catch(tf::TransformException& ex) {
                            RAVELOG_WARNA("failed to get tf frames %s (body link:%s) for object %s\n",posestamped.header.frame_id.c_str(), strrobotbaselink.c_str(), itobj->type.c_str());
                            tnew = GetTransform(itobj->pose);
                        }
                    }
                }
                else
                    tnew = GetTransform(itobj->pose);

                FOREACH(it, mapbodies) {
                    if( it->second->_initdata->sid == itobj->type ) {
                            
                        // same type matched, need to check proximity
                        if( (it->second->tnew.trans-tnew.trans).lengthsqr3() > _fThreshSqr )
                            break;

                        b = it->second;
                        mapbodies.erase(it);
                        break;
                    }
                }

                if( !b ) {
                    listnewobjs.push_back(pair<string,Transform>(itobj->type,tnew));
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

                if( !pbody->Init( itobj->first.c_str(), NULL ) ) {
                    RAVELOG_ERRORA("failed to create object %s\n", itobj->first.c_str());
                    delete pbody;
                    continue;
                }

                // append an id to the body
                wstringstream ss;
                ss << pbody->GetName() << _nNextId++;
                pbody->SetName(ss.str().c_str());

                if( !GetEnv()->AddKinBody(pbody) ) {
                    RAVELOG_ERRORA("failed to add body %S\n", pbody->GetName());
                    delete pbody;
                    continue;
                }

                BODY* b = AddKinBody(pbody, NULL);
                if( b == NULL ) {
                    GetEnv()->RemoveKinBody(pbody, true);
                    continue;
                }

                b->tnew = itobj->second;

                // put somewhere at infinity until UpdateBodies thread gets to it
                pbody->SetTransform(Transform(Vector(1,0,0,0), Vector(10000,10000,10000)));
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
    std_msgs::Pose GetPose(const Transform& t) {
        std_msgs::Pose p;
        p.orientation.x = t.rot.y; p.orientation.y = t.rot.z; p.orientation.z = t.rot.w; p.orientation.w = t.rot.x;
        p.position.x = t.trans.x; p.position.y = t.trans.y; p.position.z = t.trans.z;
        return p;
    }

    boost::shared_ptr<tf::TransformListener> _tf;
    int _robotid;
    Transform _toffset; ///< offset from tf frame
    dReal _fThreshSqr;
    int _nNextId;
};

#endif
