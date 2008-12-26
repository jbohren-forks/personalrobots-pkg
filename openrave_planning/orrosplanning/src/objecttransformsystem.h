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

#include "rossensorsystem.h"
#include "checkerboard_detector/ObjectDetection.h"

using namespace ros;

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
        : ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>(penv), nNextId(1)
    {
    }

    virtual bool Init(istream& sinput)
    {
        _topic = "ObjectDetection";
        bool bSuccess = ROSSensorSystem<checkerboard_detector::ObjectDetection, ObjectTransformXMLID>::Init(sinput);
        if( !bSuccess )
            return false;
        _fThreshSqr = 0.05*0.05f;
        sinput >> _fThreshSqr;
        return true;
    }

private:
    void newdatacb()
    {
        list< SNAPSHOT > listbodies;
        list< const checkerboard_detector::Object6DPose* > listnewobjs;

        {
            boost::mutex::scoped_lock lock(_mutex);
            TYPEOF(_mapbodies) mapbodies = _mapbodies;

            FOREACHC(itobj, _topicmsg.objects) {
                boost::shared_ptr<BODY> b;
                Transform tnew = GetTransform(itobj->pose);

                FOREACH(it, mapbodies) {
                    if( it->second->_initdata.sid == itobj->type ) {
                            
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

    Transform GetTransform(const std_msgs::Transform& pose)
    {
        return Transform(Vector(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), Vector(pose.translation.x, pose.translation.y, pose.translation.z));
    }

    dReal _fThreshSqr;
    int nNextId;
};

#endif

