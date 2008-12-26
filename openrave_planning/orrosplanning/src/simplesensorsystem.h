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
#ifndef OPENRAVE_SIMPLE_SENSOR_SYSTEM
#define OPENRAVE_SIMPLE_SENSOR_SYSTEM

using namespace std;

// used to update objects through a mocap system
template <typename XMLID>
class SimpleSensorSystem : public SensorSystemBase
{
 public:
    class MocapData : public XMLReadable
    {
    public:
        MocapData() {}
        virtual const char* GetXMLId() { return XMLID::GetXMLId(); }
    
        string sid; ///< global id for the system id 
        int id;
        string tfframe; ///< if not empty, the tf frame this object is in
        std::wstring strOffsetLink; ///< the link where the markers are attached (if any)
        Transform transOffset,transPreOffset; // final offset = transOffset * transReturnedFromVision * transPreOffset
    };

    class BODY : public BODYBASE
    {
    public:
        BODY(KinBody* pbody, const MocapData& initdata) : BODYBASE()
        {
            assert( pbody != NULL );
            _initdata = initdata;
            pOffsetLink = pbody->GetLink(_initdata.strOffsetLink.c_str());
            if( pOffsetLink == NULL )
                pOffsetLink = pbody->GetLinks().front();
            
            bPresent = false;
            bEnabled = true;
            bLock = false;
        }

        virtual void SetEnable(bool bNewEnable) { bEnabled = bNewEnable; }
        virtual void SetPresent(bool bNewPresent) { bPresent = bNewPresent; }

        virtual void* GetInitData(int* psize) { if( psize ) *psize = sizeof(_initdata); return &_initdata; }
        ros::Time lastupdated;
        MocapData _initdata;

    private:
        friend class SimpleSensorSystem<XMLID>;
    };

    class MocapXMLReader : public BaseXMLReader
    {
    public:
        MocapXMLReader(MocapData* pMocap, const char **atts) {
            _pMocap = pMocap;
            if( _pMocap == NULL )
                _pMocap = new MocapData();
        }
        virtual ~MocapXMLReader() { delete _pMocap; }
        
        void* Release() { MocapData* temp = _pMocap; _pMocap = NULL; return temp; }

        virtual void startElement(void *ctx, const char *name, const char **atts) {}
        virtual bool endElement(void *ctx, const char *name)
        {
            if( stricmp((const char*)name, XMLID::GetXMLId()) == 0 )
                return true;

            if( stricmp((const char*)name, "offsetlink") == 0 ) {
                string linkname;
                ss >> linkname;
                _pMocap->strOffsetLink = _stdmbstowcs(linkname.c_str());
            }
            else if( stricmp((const char*)name, "id") == 0 )
                ss >> _pMocap->id;
            else if( stricmp((const char*)name, "sid") == 0 )
                ss >> _pMocap->sid;
            else if( stricmp((const char*)name, "tfframe") == 0 )
                ss >> _pMocap->tfframe;
            else if( stricmp((const char*)name, "translation") == 0 )
                ss >> _pMocap->transOffset.trans.x >> _pMocap->transOffset.trans.y >> _pMocap->transOffset.trans.z;
            else if( stricmp((const char*)name, "rotationmat") == 0 ) {
                TransformMatrix m;
                ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
                _pMocap->transOffset.rot = Transform(m).rot;
            }
            else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
                Vector axis; dReal fang;
                ss >> axis.x >> axis.y >> axis.z >> fang;
                _pMocap->transOffset.rotfromaxisangle(axis,fang*(PI/180.0f));
            }
            else if( stricmp((const char*)name, "quat") == 0 )
                ss >> _pMocap->transOffset.rot;
            else if( stricmp((const char*)name, "pretranslation") == 0 )
                ss >> _pMocap->transPreOffset.trans.x >> _pMocap->transPreOffset.trans.y >> _pMocap->transPreOffset.trans.z;
            else if( stricmp((const char*)name, "prerotationmat") == 0 ) {
                TransformMatrix m;
                ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
                _pMocap->transPreOffset.rot = Transform(m).rot;
            }
            else if( stricmp((const char*)name, "prerotationaxis") == 0 ) {
                Vector axis; dReal fang;
                ss >> axis.x >> axis.y >> axis.z >> fang;
                _pMocap->transPreOffset.rotfromaxisangle(axis,fang*(PI/180.0f));
            }
            else if( stricmp((const char*)name, "prequat") == 0 )
                ss >> _pMocap->transPreOffset.rot;
            else
                RAVELOG_ERRORA("unknown field %s\n", name);

            if( !ss )
                RAVELOG_ERRORA("MocapXMLReader error parsing %s\n", name);

            return false;
        }

        virtual void characters(void *ctx, const char *ch, int len)
        {
            if( len > 0 ) {
                ss.clear();
                ss.str(string(ch, len));
            }
            else
                ss.str(""); // reset
        }

    protected:
        MocapData* _pMocap;
        stringstream ss;
    };

    static void RegisterXMLReader(EnvironmentBase* penv)
    {
        if( penv != NULL )
            penv->RegisterXMLReader(XMLID::GetXMLId(), SimpleSensorSystem<XMLID>::CreateMocapReader);
    }

    static BaseXMLReader* CreateMocapReader(KinBody* parent, const char **atts)
    {
        return new MocapXMLReader(NULL, atts);
    }

    SimpleSensorSystem(EnvironmentBase* penv) : SensorSystemBase(penv), _expirationtime(2,0)
    {
        RegisterXMLReader(GetEnv()); // just in case, register again
    }

    virtual ~SimpleSensorSystem() {
        Destroy();
    }

    virtual bool Init(istream& sinput)
    {
        return true;
    }

    virtual void Destroy()
    {
        boost::mutex::scoped_lock lock(_mutex);
        _mapbodies.clear();        
    }
    
    virtual void AddRegisteredBodies(const std::vector<KinBody*>& vbodies)
    {
        // go through all bodies in the environment and check for mocap data
        FOREACHC(itbody, vbodies) {
            MocapData* pmocapdata = (MocapData*)((*itbody)->GetExtraInterface(XMLID::GetXMLId()));
            if( pmocapdata != NULL ) {
                BODY* p = AddKinBody(*itbody, pmocapdata);
                if( p != NULL ) {
                    assert( p->GetOffsetLink() != NULL );
                    p->Lock(true);
                }
            }
        }
    }

    virtual BODY* AddKinBody(KinBody* pbody, const void* _pdata)
    {
        if( pbody == NULL )
            return false;
        assert( pbody->GetEnv() == GetEnv() );
    
        const MocapData* pdata = (const MocapData*)_pdata;
        if( pdata == NULL ) {
            pdata = (const MocapData*)(pbody->GetExtraInterface(XMLID::GetXMLId()));
            if( pdata == NULL ) {
                RAVELOG_ERRORA("failed to find mocap data for body %S\n", pbody->GetName());
                return NULL;
            }
        }

        boost::mutex::scoped_lock lock(_mutex);
        if( _mapbodies.find(pbody->GetNetworkId()) != _mapbodies.end() ) {
            RAVELOG_WARNA("body %S already added\n", pbody->GetName());
            return NULL;
        }
        
        BODY* b = new BODY(pbody, *pdata);
        _mapbodies[pbody->GetNetworkId()].reset(b);
        RAVELOG_DEBUGA("system adding body %S, total: %d\n", pbody->GetName(), _mapbodies.size());
        return b;
    }

    virtual bool RemoveKinBody(KinBody* pbody)
    {
        if( pbody == NULL )
            return false;
        assert( pbody->GetEnv() == GetEnv() );
        
        boost::mutex::scoped_lock lock(_mutex);
        bool bSuccess = _mapbodies.erase(pbody->GetNetworkId());
        RAVELOG_DEBUGA("system removing body %S %s\n", pbody->GetName(), bSuccess?"succeeded":"failed");
        return bSuccess;
    }

    virtual bool IsBodyPresent(KinBody* pbody)
    {
        if( pbody == NULL )
            return false;
        assert( pbody->GetEnv() == GetEnv() );

        boost::mutex::scoped_lock lock(_mutex);
        return _mapbodies.find(pbody->GetNetworkId()) != _mapbodies.end();
    }

    virtual bool EnableBody(KinBody* pbody, bool bEnable)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( pbody == NULL )
            return false;
        assert( pbody->GetEnv() == GetEnv() );
        
        TYPEOF(_mapbodies.begin()) it = _mapbodies.find(pbody->GetNetworkId());
        if( it == _mapbodies.end() ) {
            RAVELOG_WARNA("trying to %s body %S that is not in system\n", bEnable?"enable":"disable", pbody->GetName());
            return false;
        }

        it->second->SetEnable(bEnable);
        return true;
    }

    virtual BODY* GetBody(KinBody* pbody)
    {
        if( pbody == NULL )
            return false;
        assert( pbody->GetEnv() == GetEnv() );

        boost::mutex::scoped_lock lock(_mutex);
        TYPEOF(_mapbodies.begin()) it = _mapbodies.find(pbody->GetNetworkId());
        return it != _mapbodies.end() ? it->second.get() : NULL;
    }

    virtual bool SwitchBody(KinBody* pbody1, KinBody* pbody2)
    {
        if( pbody1 == NULL || pbody2 == NULL )
            return false;
        assert( pbody1->GetEnv() == GetEnv() && pbody2->GetEnv() == GetEnv() );

        boost::mutex::scoped_lock lock(_mutex);
        TYPEOF(_mapbodies.begin()) it = _mapbodies.find(pbody1->GetNetworkId());
        BODY* pb1 = it != _mapbodies.end() ? it->second.get() : NULL;
        it = _mapbodies.find(pbody2->GetNetworkId());
        BODY* pb2 = it != _mapbodies.end() ? it->second.get() : NULL;

        if( pb1 == NULL && pb2 == NULL )
            return false;

        if( pb1 == NULL ) {
            if( pb2->GetOffsetLink()->GetIndex() >= (int)pbody1->GetLinks().size() )
                return false;
            pb2->pOffsetLink = pbody1->GetLinks()[pb2->GetOffsetLink()->GetIndex()];
        }
        else if( pb2 == NULL ) {
            if( pb1->GetOffsetLink()->GetIndex() >= (int)pbody2->GetLinks().size() )
                return false;
            pb1->pOffsetLink = pbody2->GetLinks()[pb1->GetOffsetLink()->GetIndex()];
        }
        else
            swap(pb1->pOffsetLink, pb2->pOffsetLink);

        return true;
    }

protected:

    typedef pair<boost::shared_ptr<BODY>, Transform > SNAPSHOT;
    virtual void UpdateBodies(list<SNAPSHOT>& listbodies)
    {
        // assume mutex is already locked
        if( listbodies.size() == 0 )
            return;

        ros::Time curtime = ros::Time::now();
        GetEnv()->LockPhysics(true);

        FOREACH(it, listbodies) {
            assert( it->first->IsEnabled() && it->first->GetOffsetLink() != NULL );

            // transform with respect to offset link
            TransformMatrix tlink = it->first->GetOffsetLink()->GetTransform();
            TransformMatrix tbase = it->first->GetOffsetLink()->GetParent()->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * it->first->_initdata.transOffset;
            TransformMatrix tfinal = toffset * it->second*it->first->_initdata.transPreOffset;
            
            it->first->GetOffsetLink()->GetParent()->SetTransform(tfinal);
            it->first->lastupdated = curtime;

            //RAVELOG_DEBUGA("%f %f %f\n", tfinal.trans.x, tfinal.trans.y, tfinal.trans.z);
            
            if( !it->first->IsPresent() )
                RAVELOG_VERBOSEA("updating body %S\n", it->first->GetOffsetLink()->GetParent()->GetName());
            it->first->SetPresent(true);
        }

        GetEnv()->LockPhysics(false);

        TYPEOF(_mapbodies.begin()) itbody = _mapbodies.begin();
        while(itbody != _mapbodies.end()) {
            if( curtime-itbody->second->lastupdated > _expirationtime ) {
                if( !itbody->second->IsLocked() ) {
                    RAVELOG_DEBUGA("object %S expired %fs\n", itbody->second->GetOffsetLink()->GetParent()->GetName(), (float)(curtime-itbody->second->lastupdated).toSec());
                    GetEnv()->LockPhysics(true);
                    GetEnv()->RemoveKinBody(itbody->second->GetOffsetLink()->GetParent());
                    GetEnv()->LockPhysics(false);
                    _mapbodies.erase(itbody++);
                    continue;
                }
                
                if( itbody->second->IsPresent() )
                    RAVELOG_VERBOSEA("body %S not present\n", itbody->second->GetOffsetLink()->GetParent()->GetName());
                itbody->second->SetPresent(false);
            }

            ++itbody;
        }
    }

    map<int,boost::shared_ptr<BODY> > _mapbodies;
    boost::mutex _mutex;
    ros::Duration _expirationtime;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TEMPLATE(SimpleSensorSystem::MocapData, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(SimpleSensorSystem::BODY, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(SimpleSensorSystem::MocapXMLReader, 1)
#endif

#endif
