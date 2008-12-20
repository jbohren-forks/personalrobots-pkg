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

#include "openraveros.h"

class ROSServer : public RaveServerBase
{
    class LockEnvironment
    {
    public:
        LockEnvironment(ROSServer* pserv) : _pserv(pserv) { _pserv->GetEnv()->LockPhysics(true); }
        ~LockEnvironment() { _pserv->GetEnv()->LockPhysics(false); }

    private:
        ROSServer* _pserv;
    };

    class FIGURE
    {
    public:
        FIGURE(EnvironmentBase* penv, void* handle) : _penv(penv), _handle(handle) {}
        ~FIGURE() { _penv->closegraph(_handle); }
    private:
        FIGURE(const FIGURE& r ) { ROS_ASSERT(0); }
        EnvironmentBase* _penv;
        void* _handle;
    };

    class ServerWorker
    {
    public:
        virtual void work() = 0;
    };

    class WorkExecutor
    {
    public:
        WorkExecutor(ServerWorker* pworker) : _worker(pworker) {}
        ~WorkExecutor() { _worker->work(); }
    private:
        boost::shared_ptr<ServerWorker> _worker;
    };

    class SendProblemWorker : public ServerWorker
    {
    public:
        SendProblemWorker(boost::shared_ptr<ProblemInstance> prob, const string& cmd, string& out) : _prob(prob), _cmd(cmd), _out(out) {}
        virtual void work() {
            _prob->SendCommand(_cmd.c_str(),_out);
        }

    private:
        boost::shared_ptr<ProblemInstance> _prob;
        const string& _cmd;
        string& _out;
    };

    class LoadProblemWorker : public ServerWorker
    {
    public:
        LoadProblemWorker(boost::shared_ptr<ProblemInstance> prob, const string& args, int& retval) : _prob(prob), _args(args), _retval(retval) {}
        virtual void work() {
            _retval = _prob->GetEnv()->LoadProblem(_prob.get(), _args.c_str());
        }
    private:
        boost::shared_ptr<ProblemInstance> _prob;
        const string& _args;
        int& _retval;
    };

    class RemoveProblemWorker : public ServerWorker
    {
    public:
        RemoveProblemWorker(boost::shared_ptr<ProblemInstance> prob, bool& retval) : _prob(prob), _retval(retval) {}
        virtual void work() {
            _retval = _prob->GetEnv()->RemoveProblem(_prob.get());
        }
    private:
        boost::shared_ptr<ProblemInstance> _prob;
        bool& _retval;
    };

    class SendControllerWorker : public ServerWorker
    {
    public:
        SendControllerWorker(ControllerBase* pcontroller, const string& cmd, bool& retval) : _pcontroller(pcontroller), _cmd(cmd), _retval(retval) {}
        virtual void work() {
            _retval = _pcontroller->SendCmd(_cmd.c_str());
        }
    private:
        ControllerBase* _pcontroller;
        const string& _cmd;
        bool& _retval;
    };

    class SetControllerWorker : public ServerWorker
    {
    public:
        SetControllerWorker(RobotBase* probot, const string& type, const string& cmd, bool& retval) : _probot(probot), _type(type), _cmd(cmd), _retval(retval) {}
        virtual void work() {
            _retval = _probot->SetController(_ravembstowcs(_type.c_str()).c_str(), _cmd.c_str(), true);
        }
    private:
        RobotBase* _probot;
        const string &_type, &_cmd;
        bool& _retval;
    };

    class SendCmdSensorWorker : public ServerWorker
    {
    public:
        SendCmdSensorWorker(SensorBase* psensor, istream& is, ostream& os, bool& retval) : _psensor(psensor), _is(is), _os(os), _retval(retval) {}
        virtual void work() {
            _retval = _psensor->SendCmd(_is,_os);
        }
    private:
        SensorBase* _psensor;
        istream& _is;
        ostream& _os;
        bool& _retval;
    };

public:
        ROSServer(SetViewerFunc* psetviewer, EnvironmentBase* penv, const string& physicsengine, const string& collisionchecker, const string& viewer)
         : RaveServerBase(penv), _nNextFigureId(1), _nNextPlannerId(1), _nNextProblemId(1) {
        _psetviewer.reset(psetviewer);
        _bThreadDestroyed = false;
        _bCloseClient = false;
        _fSimulationTimestep = 0.01;
        _vgravity = Vector(0,0,-9.8f);

        SetPhysicsEngine(physicsengine);
        SetCollisionChecker(collisionchecker);
        SetViewer(viewer);
    }
    virtual ~ROSServer() {
        Destroy();
        GetEnv()->AttachServer(NULL);
    }
    
    virtual void Destroy()
    {
        _bCloseClient = true;
        Reset();
        
        GetEnv()->SetCollisionChecker(NULL);
        GetEnv()->SetPhysicsEngine(NULL);
        GetEnv()->AttachViewer(NULL);
        
        // have to maintain a certain destruction order
        _pphysics.reset();
        _pcolchecker.reset();
        _threadviewer.join();

        _bCloseClient = false;
    }

    virtual void Reset()
    {
        Worker();

        _mapFigureIds.clear();

        // destroy environment specific state: problems, planners, figures
        _mapplanners.clear();
        _mapproblems.clear();
    }

    virtual bool Init(int port)
    {
        return true;
    }

    /// worker thread called from the main environment thread
    virtual void Worker()
    {
        list<boost::shared_ptr<WorkExecutor> > listlocalworkers;
        {
            boost::mutex::scoped_lock lock(_mutexWorker);
            listlocalworkers.swap(_listWorkers);
        }

        listlocalworkers.clear();
        _conditionWorkers.notify_all();
    }

    virtual void AddWorker(ServerWorker* pworker, bool bWait=true)
    {
        boost::mutex::scoped_lock lock(_mutexWorker);
        _listWorkers.push_back(boost::shared_ptr<WorkExecutor>(new WorkExecutor(pworker)));
        if( bWait )
            _conditionWorkers.wait(lock);
    }

    virtual bool IsInit()
    {
        return true;
    }

    virtual bool IsClosing()
    {
        return _bCloseClient;
    }

    /// viewer thread assuming you can create different viewers in their own therads
    virtual void ViewerThread(const string& strviewer)
    {
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _pviewer.reset(GetEnv()->CreateViewer(strviewer.c_str()));
            if( !!_pviewer ) {
                GetEnv()->AttachViewer(_pviewer.get());
                _pviewer->ViewerSetSize(1024,768);
            }
            _conditionViewer.notify_all();
        }

        if( !_pviewer )
            return;

        _pviewer->main(); // spin until quitfrommainloop is called
        RAVELOG_DEBUGA("destroying viewer\n");
        _pviewer.reset();
    }

    bool SetPhysicsEngine(const string& physicsengine)
    {
        if( physicsengine.size() > 0 ) {
            _pphysics.reset(GetEnv()->CreatePhysicsEngine(physicsengine.c_str()));
            GetEnv()->SetPhysicsEngine(_pphysics.get());
            if( !_pphysics )
                return false;
            _pphysics->SetGravity(_vgravity);
        }

        return true;
    }

    bool SetCollisionChecker(const string& collisionchecker)
    {
        if( collisionchecker.size() > 0 ) {
            _pcolchecker.reset(GetEnv()->CreateCollisionChecker(collisionchecker.c_str()));
            if( !_pcolchecker )
                return false;
            GetEnv()->SetCollisionChecker(_pcolchecker.get());
        }

        return true;
    }

    bool SetViewer(const string& viewer)
    {
        if( !!_psetviewer )
            return _psetviewer->SetViewer(GetEnv(),viewer);

        _threadviewer.join(); // wait for the viewer
        
        if( viewer.size() > 0 ) {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _threadviewer = boost::thread(boost::bind(&ROSServer::ViewerThread, this, viewer));
            _conditionViewer.wait(lock);
            
            if( !_pviewer ) {
                RAVELOG_WARNA("failed to create viewer %s\n", viewer.c_str());
                _threadviewer.join();
                return false;
            }
            else
                RAVELOG_INFOA("viewer %s successfully attached\n", viewer.c_str());
        }

        return true;
    }

    //////////////
    // services //
    //////////////

    bool body_destroy_srv(body_destroy::request& req, body_destroy::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        LockEnvironment envlock(this);
        return GetEnv()->RemoveKinBody(pbody,true);
    }

    bool body_enable_srv(body_enable::request& req, body_enable::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        LockEnvironment envlock(this);
        pbody->Enable(req.enable);
        return true;
    }

    bool body_getaabb_srv(body_getaabb::request& req, body_getaabb::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        LockEnvironment envlock(this);
        OpenRAVE::AABB ab = pbody->ComputeAABB();
        res.box.center[0] = ab.pos.x; res.box.center[1] = ab.pos.y; res.box.center[2] = ab.pos.z;
        res.box.extents[0] = ab.extents.x; res.box.extents[1] = ab.extents.y; res.box.extents[2] = ab.extents.z;
        return true;
    }

    bool body_getaabbs_srv(body_getaabbs::request& req, body_getaabbs::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        LockEnvironment envlock(this);
        res.boxes.resize(pbody->GetLinks().size()); int index = 0;
        FOREACHC(itlink, pbody->GetLinks()) {
            OpenRAVE::AABB ab = (*itlink)->ComputeAABB();
            openraveros::AABB& resbox = res.boxes[index++];
            resbox.center[0] = ab.pos.x; resbox.center[1] = ab.pos.y; resbox.center[2] = ab.pos.z;
            resbox.extents[0] = ab.extents.x; resbox.extents[1] = ab.extents.y; resbox.extents[2] = ab.extents.z;
        }
        return true;
    }

    bool body_getdof_srv(body_getdof::request& req, body_getdof::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;
        res.dof = pbody->GetDOF();
        return true;
    }

    bool body_getjointvalues_srv(body_getjointvalues::request& req, body_getjointvalues::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;
        LockEnvironment envlock(this);

        if( req.indices.size() > 0 ) {
            vector<dReal> vtemp;
            pbody->GetJointValues(vtemp);
            
            res.values.resize(req.indices.size());
            for(size_t i = 0; i < req.indices.size(); ++i) {
                ROS_ASSERT( req.indices[i] < vtemp.size() );
                res.values[i] = vtemp[req.indices[i]];
            }
        }
        else {
            vector<dReal> vtemp;
            pbody->GetJointValues(vtemp);

            res.values.resize(vtemp.size());
            for(size_t i = 0; i < res.values.size(); ++i)
                res.values[i] = vtemp[i];
        }
        
        return true;
    }

    bool body_setjointvalues_srv(body_setjointvalues::request& req, body_setjointvalues::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        if( pbody->GetDOF() == 0 )
            return false;

        LockEnvironment envlock(this);
        vector<dReal> vvalues;

        if( req.indices.size() > 0 ) {
            if( req.indices.size() != req.jointvalues.size() ) {
                RAVELOG_WARNA("indices (%d) different size than joint values (%d)\n", req.indices.size(), req.jointvalues.size());
                return false;
            }

            pbody->GetJointValues(vvalues);
            for(uint32_t i = 0; i < req.indices.size(); ++i)
                vvalues[req.indices[i]] = req.jointvalues[i];
        }
        else {
            if( pbody->GetDOF() != (int)req.jointvalues.size() ) {
                RAVELOG_WARNA("body dof (%d) not equal to jointvalues (%d)", pbody->GetDOF(), req.jointvalues.size());
                return false;
            }

            vvalues.reserve(req.jointvalues.size());
            FOREACHC(it,req.jointvalues)
                vvalues.push_back(*it);
        }

        pbody->SetJointValues(NULL, NULL, &vvalues[0], true);

        if( pbody->IsRobot() ) {
            // if robot, should turn off any trajectory following
            RobotBase* probot = (RobotBase*)pbody;
            if( probot->GetController() != NULL ) {
                probot->GetJointValues(vvalues); // reget the values since they'll go through the joint limits
                probot->GetController()->SetDesired(&vvalues[0]);
            }
        }

        return true;
    }

    bool body_settransform_srv(body_settransform::request& req, body_settransform::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        Transform t = FromAffineTransform(req.transform);
        LockEnvironment envlock(this);
        pbody->SetTransform(t);

        if( pbody->IsRobot() ) {
            RobotBase* probot = (RobotBase*)pbody;
            if( probot->GetController() != NULL )
                probot->GetController()->SetPath(NULL);
        }

        return true;
    }

    bool env_checkcollision_srv(env_checkcollision::request& req, env_checkcollision::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL )
            return false;

        set<KinBody*> setignore;
        FOREACH(it, req.excludedbodyids) {
            KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
            if( pbody == NULL )
                return false;
            setignore.insert(pbody);
        }

        COLLISIONREPORT report;
        {
            LockEnvironment envlock(this);
            int oldopts = GetEnv()->GetCollisionOptions();
            if( !GetEnv()->SetCollisionOptions(req.options) )
                RAVELOG_WARNA("failed to set collision options\n");
            
            set<KinBody::Link*> empty;
            if( req.linkid < 0 )
                res.collision = GetEnv()->CheckCollision(pbody, setignore, empty, &report);
            else {
                if( req.linkid >= (int)pbody->GetLinks().size() )
                    return false;
                res.collision = GetEnv()->CheckCollision(pbody->GetLinks()[req.linkid], setignore, empty, &report);
            }

            GetEnv()->SetCollisionOptions(oldopts);
        }

        res.collidingbodyid = 0;

        if( res.collision ) {
            KinBody::Link* plinkcol = report.plink1;
            if( report.plink2 && report.plink2->GetParent() != pbody ) {
                ROS_ASSERT(report.plink1->GetParent() == pbody);
                plinkcol = report.plink2;
            }

            if( plinkcol != NULL ) {
                res.collidingbodyid = plinkcol->GetParent()->GetNetworkId();
                res.collidinglink = plinkcol->GetIndex();
            }

            RAVELOG_INFOA("collision %S:%S with %S:%S\n",
                       report.plink1?report.plink1->GetParent()->GetName():L"(NULL",
                       report.plink1?report.plink1->GetName():L"(NULL)",
                       report.plink2?report.plink2->GetParent()->GetName():L"(NULL)",
                       report.plink2?report.plink2->GetName():L"(NULL)");
        }

        if( req.options & CO_Distance )
            res.mindist = report.minDistance;
        if( req.options & CO_Contacts ) {
            res.contacts.resize(report.contacts.size()); int index = 0;
            FOREACHC(itc, report.contacts) {
                openraveros::Contact& c = res.contacts[index++];
                c.position[0] = itc->pos.x; c.position[1] = itc->pos.y; c.position[2] = itc->pos.z;
                c.normal[0] = itc->norm.x; c.normal[1] = itc->norm.y; c.normal[2] = itc->norm.z;
            }
        }

        return true;
    }

    bool env_closefigures_srv(env_closefigures::request& req, env_closefigures::response& res)
    {
        bool bSuccess = true;

        if( req.figureids.size() > 0 ) {
            FOREACH(itid, req.figureids) {
                if( !_mapFigureIds.erase(*itid) )
                    bSuccess = false;
            }
        }
        else // destroy everything
            _mapFigureIds.clear();

        return bSuccess;
    }

    bool env_createbody_srv(env_createbody::request& req, env_createbody::response& res)
    {
        LockEnvironment envlock(this);
        KinBody* pbody = GetEnv()->CreateKinBody();

        if( req.file.size() > 0 ) {
            if( !pbody->Init(req.file.c_str(), NULL) ) {
                delete pbody;
                return false;
            }
        }

        pbody->SetName(req.name.c_str());

        if( !GetEnv()->AddKinBody(pbody) ) {
            delete pbody;
            return false;
        }

        res.bodyid = pbody->GetNetworkId();
        return true;
    }

    bool env_createplanner_srv(env_createplanner::request& req, env_createplanner::response& res)
    {
        boost::shared_ptr<PlannerBase> pplanner(GetEnv()->CreatePlanner(req.plannertype.c_str()));
        if( !pplanner )
            return false;

        _mapplanners[++_nNextPlannerId] = pplanner;
        res.plannerid = _nNextPlannerId;
        return true;
    }

    bool env_createproblem_srv(env_createproblem::request& req, env_createproblem::response& res)
    {
        boost::shared_ptr<ProblemInstance> pproblem(GetEnv()->CreateProblem(req.problemtype.c_str()));
        if( !pproblem )
            return false;

        if( req.destroyduplicates ) {
            map<int, boost::shared_ptr<ProblemInstance> >::iterator itprob = _mapproblems.begin();
            while(itprob != _mapproblems.end()) {
                if( stricmp(itprob->second->GetXMLId(), req.problemtype.c_str()) == 0 ) {
                    RAVELOG_INFOA("deleting duplicate problem %s\n", req.problemtype.c_str());
                    if( !GetEnv()->RemoveProblem(itprob->second.get()) )
                        RAVELOG_WARNA("failed to remove problem %s\n", itprob->second->GetXMLId());
                    
                    _mapproblems.erase(itprob++);
                }
                else
                    ++itprob;
            }
        }

        int retval=0;
        AddWorker(new LoadProblemWorker(pproblem, req.args, retval), true);

        if( retval != 0 ) {
            RAVELOG_WARNA("failed to load problem %s with args %s\n", req.problemtype.c_str(), req.args.c_str());
            return false;
        }
        
        _mapproblems[++_nNextProblemId] = pproblem;
        res.problemid = _nNextProblemId;
        return true;
    }

    bool env_createrobot_srv(env_createrobot::request& req, env_createrobot::response& res)
    {
        RobotBase* probot = GetEnv()->CreateRobot(req.type.c_str());
        if( !probot )
            return false;

        if( req.file.size() > 0 ) {
            if( !probot->Init(req.file.c_str(), NULL) )
                return false;
        }

        probot->SetName(req.name.c_str());

        if( !GetEnv()->AddRobot(probot) )
            return false;

        res.bodyid = probot->GetNetworkId();
        return true;
    }

    bool env_destroyproblem_srv(env_destroyproblem::request& req, env_destroyproblem::response& res)
    {
        map<int, boost::shared_ptr<ProblemInstance> >::iterator itprob = _mapproblems.find(req.problemid);
        if( itprob == _mapproblems.end() )
            return false;

        bool bsuccess=false;
        AddWorker(new RemoveProblemWorker(itprob->second,bsuccess), true);
        if( !bsuccess )
            RAVELOG_WARNA("failed to remove problem\n");

        _mapproblems.erase(itprob);
        return bsuccess;
    }

    void FillKinBodyInfo(KinBody* pbody, BodyInfo& info, uint32_t options)
    {
        info.bodyid = pbody->GetNetworkId();
        info.transform = GetAffineTransform(pbody->GetTransform());
        info.enabled = pbody->IsEnabled();

        if( options & BodyInfo::Req_JointValues ) {
            vector<dReal> vvalues; pbody->GetJointValues(vvalues);
            info.jointvalues.resize(vvalues.size());
            for(size_t i = 0; i < vvalues.size(); ++i)
                info.jointvalues[i] = vvalues[i];
        }
        if( options & BodyInfo::Req_Links ) {
            vector<Transform> vlinks; pbody->GetBodyTransformations(vlinks);
            info.links.resize(vlinks.size());
            for(size_t i = 0; i < vlinks.size(); ++i)
                info.links[i] = GetAffineTransform(vlinks[i]);
        }
        if( options & BodyInfo::Req_LinkNames ) {
            info.linknames.resize(pbody->GetLinks().size());
            for(size_t i = 0; i < info.linknames.size(); ++i)
                info.linknames[i] = _stdwcstombs(pbody->GetLinks()[i]->GetName());
        }
        if( options & BodyInfo::Req_JointLimits ) {
            vector<dReal> vlower, vupper;
            pbody->GetJointLimits(vlower,vupper);
            ROS_ASSERT( vlower.size() == vupper.size() );
            info.lowerlimit.resize(vlower.size());
            info.upperlimit.resize(vupper.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                info.lowerlimit[i] = vlower[i];
                info.upperlimit[i] = vupper[i];
            }
        }
        if( options & BodyInfo::Req_Names ) {
            info.filename = pbody->GetXMLFilename();
            info.name = _stdwcstombs(pbody->GetName());
            info.type = pbody->GetXMLId();
        }
    }

    bool env_getbodies_srv(env_getbodies::request& req, env_getbodies::response& res)
    {
        vector<KinBody*> vbodies;
        boost::shared_ptr<EnvironmentBase::EnvLock> lock(GetEnv()->GetLockedBodies(vbodies));
        LockEnvironment envlock(this);

        if( req.bodyid != 0 ) {
            KinBody* pfound = NULL;
            FOREACH(it, vbodies) {
                if( (*it)->GetNetworkId() == req.bodyid ) {
                    pfound = *it;
                    break;
                }
            }

            if( pfound == NULL )
                return false;

            // add only one body
            vbodies.resize(0); vbodies.push_back(pfound);
        }

        res.bodies.resize(vbodies.size()); int index = 0;
        FOREACH(itbody, vbodies) {
            openraveros::BodyInfo& info = res.bodies[index++];
            FillKinBodyInfo(*itbody, info, req.options);
        }
        
        return true;
    }

    bool env_getbody_srv(env_getbody::request& req, env_getbody::response& res)
    {
        KinBody* pbody = GetEnv()->GetKinBody(_ravembstowcs(req.name.c_str()).c_str());
        if( pbody == NULL )
            return false;
        res.bodyid = pbody->GetNetworkId();
        return true;
    }

    void FillActiveDOFs(RobotBase* probot, openraveros::ActiveDOFs& active)
    {
        active.affine = probot->GetAffineDOF();
        active.joints.resize(probot->GetActiveJointIndices().size());
        for(size_t i = 0; i < probot->GetActiveJointIndices().size(); ++i)
            active.joints[i] = probot->GetActiveJointIndices()[i];
        active.rotationaxis[0] = probot->GetAffineRotationAxis().x;
        active.rotationaxis[1] = probot->GetAffineRotationAxis().y;
        active.rotationaxis[2] = probot->GetAffineRotationAxis().z;
    }

    void FillRobotInfo(RobotBase* probot, RobotInfo& info, uint32_t options)
    {
        FillKinBodyInfo(probot,info.bodyinfo,options);

        info.activedof = probot->GetActiveDOF();
        info.activemanip = probot->GetActiveManipulatorIndex();

        if( options & RobotInfo::Req_Manipulators ) {
            info.manips.resize(probot->GetManipulators().size()); int index = 0;
            FOREACHC(itmanip, probot->GetManipulators()) {
                openraveros::Manipulator& rosmanip = info.manips[index++];
                rosmanip.baselink = itmanip->pBase != NULL ? itmanip->pBase->GetIndex() : -1;
                rosmanip.eelink = itmanip->pEndEffector != NULL ? itmanip->pEndEffector->GetIndex() : -1;
                rosmanip.tgrasp = GetAffineTransform(itmanip->tGrasp);
                
                rosmanip.handjoints.resize(itmanip->_vecjoints.size());
                for(size_t i = 0; i < itmanip->_vecjoints.size(); ++i)
                    rosmanip.handjoints[i] = itmanip->_vecjoints[i];

                rosmanip.armjoints.resize(itmanip->_vecarmjoints.size());
                for(size_t i = 0; i < itmanip->_vecarmjoints.size(); ++i)
                    rosmanip.armjoints[i] = itmanip->_vecarmjoints[i];

                rosmanip.iksolvername = _stdwcstombs(itmanip->GetIKSolverName().c_str());
            }
        }
        if( options & RobotInfo::Req_Sensors ) {
            info.sensors.resize(probot->GetSensors().size()); int index = 0;
            FOREACHC(its, probot->GetSensors()) {
                openraveros::AttachedSensor& rossensor = info.sensors[index++];
                rossensor.name = _stdwcstombs(its->GetName());
                rossensor.attachedlink = its->GetAttachingLink()->GetIndex();
                rossensor.trelative = GetAffineTransform(its->GetRelativeTransform());

                if( its->GetSensor() != NULL ) {
                    rossensor.tglobal = GetAffineTransform(its->GetSensor()->GetTransform());
                    rossensor.type = its->GetSensor()->GetXMLId();
                }
                else {
                    rossensor.tglobal = GetAffineIdentity();
                    rossensor.type = "";
                }
            }
        }
        if( options & RobotInfo::Req_ActiveDOFs ) {
            FillActiveDOFs(probot, info.active);
        }
        if( options & RobotInfo::Req_ActiveLimits ) {
            vector<dReal> vlower, vupper;
            probot->GetActiveDOFLimits(vlower,vupper);
            ROS_ASSERT( vlower.size() == vupper.size() );
            info.activelowerlimit.resize(vlower.size());
            info.activeupperlimit.resize(vupper.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                info.activelowerlimit[i] = vlower[i];
                info.activeupperlimit[i] = vupper[i];
            }
        }
    }

    bool env_getrobots_srv(env_getrobots::request& req, env_getrobots::response& res)
    {
        vector<RobotBase*> vrobots;
        boost::shared_ptr<EnvironmentBase::EnvLock> lock(GetEnv()->GetLockedRobots(vrobots));
        LockEnvironment envlock(this);

        if( req.bodyid != 0 ) {
            RobotBase* pfound = NULL;
            FOREACH(it, vrobots) {
                if( (*it)->GetNetworkId() == req.bodyid ) {
                    pfound = *it;
                    break;
                }
            }

            if( pfound == NULL )
                return false;

            // add only one body
            vrobots.resize(0); vrobots.push_back(pfound);
        }

        res.robots.resize(vrobots.size()); int index = 0;
        FOREACH(itrobot, vrobots) {
            ROS_ASSERT( (*itrobot)->IsRobot() );
            openraveros::RobotInfo& info = res.robots[index++];
            FillRobotInfo(*itrobot, info, req.options);
        }
        
        return true;
    }

    bool env_loadplugin_srv(env_loadplugin::request& req, env_loadplugin::response& res)
    {
        LockEnvironment envlock(this);
        return GetEnv()->LoadPlugin(req.filename.c_str());
    }

    bool env_loadscene_srv(env_loadscene::request& req, env_loadscene::response& res)
    {
        if( req.resetscene ) {
            RAVELOG_INFOA("resetting scene\n");
            GetEnv()->Reset();
        }
        
        if( req.filename.size() > 0 ) {
            LockEnvironment envlock(this);
            return GetEnv()->Load(req.filename.c_str());
        }

        return true;
    }

    bool env_plot_srv(env_plot::request& req, env_plot::response& res)
    {
        bool bOneColor = req.colors.size() != req.points.size();
        float falpha = max(0.0f, 1.0f-req.transparency);
        falpha = min(1.0f,falpha);
        RaveVector<float> vOneColor(1.0f,0.5f,0.5f,falpha);
        if( req.colors.size() >= 3 )
            vOneColor = RaveVector<float>(req.colors[0], req.colors[1], req.colors[2],falpha);
        
        void* figure = NULL;
        switch(req.drawtype) {
        case env_plot::request::Draw_Point:
            if( bOneColor )
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 0);
            else
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            break;
        case env_plot::request::Draw_LineStrip:
            if( bOneColor )
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            else
                figure = GetEnv()->drawlinestrip(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            break;
        case env_plot::request::Draw_LineList:
            if( bOneColor )
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor);
            else
                figure = GetEnv()->drawlinelist(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0]);
            break;
        case env_plot::request::Draw_TriList:
            //if( bOneColor )
            figure = GetEnv()->drawtrimesh(&req.points[0],3*sizeof(req.points[0]), NULL, req.points.size()/9, vOneColor);
            //else
                //figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 0);
            break;
        case env_plot::request::Draw_Sphere:
            if( bOneColor )
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,vOneColor, 1);
            else
                figure = GetEnv()->plot3(&req.points[0],req.points.size()/3,3*sizeof(req.points[0]),req.size,&req.colors[0], 1);
            break;
        default:
            return false;
        }

        if( figure == NULL )
            return false;
        
        _mapFigureIds[++_nNextFigureId].reset(new FIGURE(GetEnv(), figure));
        res.figureid = _nNextFigureId;
        return true;
    }

    bool env_raycollision_srv(env_raycollision::request& req, env_raycollision::response& res)
    {
        KinBody* pbody = req.bodyid != 0 ? GetEnv()->GetBodyFromNetworkId(req.bodyid) : NULL;
        
        res.collision.reserve(req.rays.size()); res.collision.resize(0);

        if( req.request_contacts ) {
            res.contacts.reserve(req.rays.size());
            res.contacts.resize(0);
        }

        if( req.request_bodies ) {
            res.hitbodies.reserve(req.rays.size());
            res.hitbodies.resize(0);
        }

        COLLISIONREPORT report;
        {
            LockEnvironment envlock(this);
            int oldopts = GetEnv()->GetCollisionOptions();
            if( !GetEnv()->SetCollisionOptions(req.request_contacts ? CO_Contacts : 0) )
                RAVELOG_WARNA("failed to set collision options\n");

            FOREACHC(itray, req.rays) {
                RAY r;
                r.pos = Vector(itray->position[0], itray->position[1], itray->position[2]);
                r.dir = Vector(itray->direction[0], itray->direction[1], itray->direction[2]);

                uint8_t bCollision = 0;

                if( r.dir.lengthsqr3() > 1e-7 ) {
                    r.dir.normalize3();
                
                    if( pbody != NULL )
                        bCollision = GetEnv()->CheckCollision(r,pbody,&report);
                    else
                        bCollision = GetEnv()->CheckCollision(r,&report);
                }
                else
                    RAVELOG_WARNA("ray has zero direction\n");
                
                res.collision.push_back(bCollision);
                if( bCollision && req.request_contacts ) {
                    openraveros::Contact rosc;
                    if( report.contacts.size() > 0 ) {
                        COLLISIONREPORT::CONTACT& c = report.contacts.front();
                        rosc.position[0] = c.pos.x; rosc.position[1] = c.pos.y; rosc.position[2] = c.pos.z;
                        rosc.normal[0] = c.norm.x; rosc.normal[1] = c.norm.y; rosc.normal[2] = c.norm.z;
                    }
                    res.contacts.push_back(rosc);
                }
                else
                    res.contacts.push_back(openraveros::Contact());
                    

                if( bCollision && req.request_bodies ) {
                    KinBody::Link* plink = report.plink1 != NULL ? report.plink1 : report.plink2;
                    res.hitbodies.push_back(plink != NULL ? plink->GetParent()->GetNetworkId() : 0);
                }
                else
                    res.hitbodies.push_back(0);
            }

            GetEnv()->SetCollisionOptions(oldopts);
        }

        return true;
    }

    bool env_set_srv(env_set::request& req, env_set::response& res)
    {
        if( req.setmask & env_set::request::Set_Simulation ) {
            switch(req.sim_action) {
            case env_set::request::SimAction_Start:
                if( req.sim_timestep > 0 )
                    _fSimulationTimestep = req.sim_timestep;
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            case env_set::request::SimAction_Stop:
                GetEnv()->StopSimulation();
                break;
            case env_set::request::SimAction_Timestep:
                _fSimulationTimestep = req.sim_timestep;
                GetEnv()->StartSimulation(_fSimulationTimestep);
                break;
            }
        }
        if( req.setmask & env_set::request::Set_PhysicsEngine ) {
            SetPhysicsEngine(req.physicsengine);
        }
        if( req.setmask & env_set::request::Set_CollisionChecker ) {
            int options = GetEnv()->GetCollisionOptions();
            if( SetCollisionChecker(req.collisionchecker) )
                GetEnv()->SetCollisionOptions(options);
        }
        if( req.setmask & env_set::request::Set_Gravity ) {
            _vgravity = Vector(req.gravity[0],req.gravity[1],req.gravity[2]);
            if( !!_pphysics )
                _pphysics->SetGravity(_vgravity);
        }
        if( req.setmask & env_set::request::Set_PublishAnytime ) {
            GetEnv()->SetPublishBodiesAnytime(req.publishanytime>0);
        }
        if( req.setmask & env_set::request::Set_DebugLevel ) {
            map<string,DebugLevel> mlevels;
            mlevels["fatal"] = Level_Fatal;
            mlevels["error"] = Level_Error;
            mlevels["info"] = Level_Info;
            mlevels["warn"] = Level_Warn;
            mlevels["debug"] = Level_Debug;
            mlevels["verbose"] = Level_Verbose;
            DebugLevel level = GetEnv()->GetDebugLevel();
            if( mlevels.find(req.debuglevel) != mlevels.end() )
                level = mlevels[req.debuglevel];
            else {
                stringstream ss(req.debuglevel);
                int nlevel;
                ss >> nlevel;
                if( !!ss )
                    level = (DebugLevel)nlevel;
            }
            GetEnv()->SetDebugLevel(level);
        }
        if( req.setmask & env_set::request::Set_Viewer ) {
            GetEnv()->AttachViewer(NULL);
            SetViewer(req.viewer);
        }

        return true;
    }

    bool env_triangulate_srv(env_triangulate::request& req, env_triangulate::response& res)
    {
        set<int> setobjids;
        FOREACH(it, req.bodyids)
            setobjids.insert(*it);
        
        KinBody::Link::TRIMESH trimesh;

        {
            LockEnvironment envlock(this);
            
            vector<KinBody*> vbodies;
            boost::shared_ptr<EnvironmentBase::EnvLock> lock(GetEnv()->GetLockedBodies(vbodies));

            FOREACH(itbody, vbodies) {
                if( (setobjids.find((*itbody)->GetNetworkId()) == setobjids.end()) ^ !req.inclusive )
                    continue;
                GetEnv()->Triangulate(trimesh, *itbody);
            }
        }

        res.points.resize(3*trimesh.vertices.size());
        for(size_t i = 0; i < trimesh.vertices.size(); ++i) {
            Vector& v = trimesh.vertices[i];
            res.points[3*i+0] = v.x;
            res.points[3*i+1] = v.y;
            res.points[3*i+2] = v.z;
        }

        res.indices.resize(trimesh.indices.size());
        for(size_t i = 0; i < trimesh.indices.size(); ++i)
            res.indices[i] = (uint32_t)trimesh.indices[i];

        return true;
    }

    bool env_wait_srv(env_wait::request& req, env_wait::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;
        
        if( probot->GetController() == NULL )
            return false;

        uint32_t timeout = (uint32_t)(req.timeout*1000.0f);
        
        while( !probot->GetController()->IsDone() ) {
            usleep(400);
            if( timeout > 0 ) {
                if( --timeout == 0 )
                    break;
            }
            if( IsClosing() )
                return false;
        }

        res.isdone = probot->GetController()->IsDone();
        return true;
    }

    bool planner_init_srv(planner_init::request& req, planner_init::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.robotid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        map<int, boost::shared_ptr<PlannerBase> >::iterator itplanner = _mapplanners.find(req.plannerid);
        if( itplanner == _mapplanners.end() )
            return false;
        
        PlannerBase::PlannerParameters params;
        // fill with request
        RAVELOG_ERRORA("need to fill with params!\n");

        LockEnvironment envlock(this);
        return itplanner->second->InitPlan(probot, &params);
    }

    bool planner_plan_srv(planner_plan::request& req, planner_plan::response& res)
    {
        map<int, boost::shared_ptr<PlannerBase> >::iterator itplanner = _mapplanners.find(req.plannerid);
        if( itplanner == _mapplanners.end() )
            return false;

        ROS_ASSERT( itplanner->second->GetRobot() != NULL );
        LockEnvironment envlock(this);

        boost::shared_ptr<OpenRAVE::Trajectory> traj(GetEnv()->CreateTrajectory(itplanner->second->GetRobot()->GetActiveDOF()));
        
        RAVELOG_INFOA("starting to plan");
        if( !itplanner->second->PlanPath(traj.get(), NULL) ) {
            RAVELOG_INFOA("plan failed\n");
            return false;
        }
        
        RAVELOG_INFOA("plan succeeded\n");

        FillActiveDOFs(itplanner->second->GetRobot(), res.trajectory.active);
        res.trajectory.points.resize(traj->GetPoints().size()); int index = 0;
        FOREACHC(itpoint, traj->GetPoints()) {
            openraveros::TrajectoryPoint& rospt = res.trajectory.points[index++];
            rospt.timestamp = itpoint->time;
            rospt.position.resize(itpoint->q.size());
            for(size_t i = 0; i < itpoint->q.size(); ++i)
                rospt.position[i] = itpoint->q[i];
            rospt.velocity.resize(itpoint->qdot.size());
            for(size_t i = 0; i < itpoint->qdot.size(); ++i)
                rospt.velocity[i] = itpoint->qdot[i];
            rospt.transform = GetAffineTransform(itpoint->trans);
        }

        return true;
    }

    bool problem_sendcommand_srv(problem_sendcommand::request& req, problem_sendcommand::response& res)
    {
        map<int, boost::shared_ptr<ProblemInstance> >::iterator itprob = _mapproblems.find(req.problemid);
        if( itprob == _mapproblems.end() )
            return false;

        AddWorker(new SendProblemWorker(itprob->second,req.cmd,res.output), true);
        return true;
    }

    bool robot_controllersend_srv(robot_controllersend::request& req, robot_controllersend::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        if( probot->GetController() == NULL || !probot->GetController()->SupportsCmd(req.cmd.c_str()) )
            return false;

        bool bsuccess = false;
        AddWorker(new SendControllerWorker(probot->GetController(), req.cmd, bsuccess), true);
        return bsuccess;
    }

    bool robot_controllerset_srv(robot_controllerset::request& req, robot_controllerset::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        bool bsuccess = false;
        AddWorker(new SetControllerWorker((RobotBase*)pbody, req.controllername, req.controllerargs, bsuccess), true);
        return bsuccess;
    }

    bool robot_getactivevalues_srv(robot_getactivevalues::request& req, robot_getactivevalues::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        LockEnvironment envlock(this);

        if( req.indices.size() > 0 ) {
            vector<dReal> vtemp;
            probot->GetActiveDOFValues(vtemp);
            
            res.values.resize(req.indices.size());
            for(size_t i = 0; i < req.indices.size(); ++i) {
                ROS_ASSERT( req.indices[i] < vtemp.size() );
                res.values[i] = vtemp[req.indices[i]];
            }
        }
        else {
            vector<dReal> vtemp;
            probot->GetActiveDOFValues(vtemp);

            res.values.resize(vtemp.size());
            for(size_t i = 0; i < res.values.size(); ++i)
                res.values[i] = vtemp[i];
        }

        return true;
    }

    bool robot_sensorgetdata_srv(robot_sensorgetdata::request& req, robot_sensorgetdata::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        if( req.sensorindex >= probot->GetSensors().size() )
            return false;


        if( probot->GetSensors()[req.sensorindex].GetSensor() == NULL )
            return false;

        SensorBase* psensor = probot->GetSensors()[req.sensorindex].GetSensor();

        boost::shared_ptr<SensorBase::SensorData> pdata(psensor->CreateSensorData());

        if( !pdata ) {
            RAVELOG_ERRORA("Robot %S, failed to create sensor %S data\n", probot->GetName(), probot->GetSensors()[req.sensorindex].GetName());
            return false;
        }

        if( !psensor->GetSensorData(pdata.get()) ) {
            RAVELOG_ERRORA("Robot %S, failed to get sensor %S data\n", probot->GetName(), probot->GetSensors()[req.sensorindex].GetName());
            return false;
        }

        // serialize the data

        switch(pdata->GetType()) {
        case SensorBase::ST_Laser: {
            res.type = "laser";
            SensorBase::LaserSensorData* plaserdata = (SensorBase::LaserSensorData*)pdata.get();
            
            int index;

            res.laserrange.resize(3*plaserdata->ranges.size()); index = 0;
            FOREACH(itpos, plaserdata->ranges) {
                res.laserrange[3*index+0] = itpos->x;
                res.laserrange[3*index+1] = itpos->y;
                res.laserrange[3*index+2] = itpos->z;
                ++index;
            }

            res.laserpos.resize(3*plaserdata->positions.size()); index = 0;
            FOREACH(itpos, plaserdata->positions) {
                res.laserpos[3*index+0] = itpos->x;
                res.laserpos[3*index+1] = itpos->y;
                res.laserpos[3*index+2] = itpos->z;
                ++index;
            }
        
            res.laserint.resize(plaserdata->intensity.size()); index = 0;
            FOREACH(itint, plaserdata->intensity)
                res.laserint[index++] = *itint;

            break;
        }
        case SensorBase::ST_Camera:
            res.type = "camera";
            RAVELOG_ERRORA("camera data serialization not implemented yet!\n");
            ROS_ASSERT(0);
            break;
//            SensorBase::CameraSensorData* pcameradata = (SensorBase::CameraSensorData*)pdata.get();
//
//            if( psensor->GetSensorGeometry()->GetType() != SensorBase::ST_Camera ) {
//                RAVELOG(L"sensor geometry not a camera type\n");
//                return false;
//            }
//
//            SensorBase::CameraGeomData* pgeom = (SensorBase::CameraGeomData*)psensor->GetSensorGeometry();
//
//            if( (int)pcameradata->vimagedata.size() != pgeom->width*pgeom->height*3 ) {
//                RAVELOG(L"image data wrong size %d != %d\n", pcameradata->vimagedata.size(), pgeom->width*pgeom->height*3);
//                return false;
//            }
//
//            sout << pgeom->width << " " << pgeom->height << " ";
//            for(int i = 0; i < 4; ++i)
//                sout << pgeom->KK[i] << " ";
//            sout << TransformMatrix(pcameradata->t) << " ";
//            //FOREACH(it, pcameradata->vimagedata) sout << (int)*it << " ";
//
//            // RLE encoding (about 3x faster than sending raw images)
//            int curvalue = 0, lastdiff = 0, lastvalue = 0xffffff&*(int*)&pcameradata->vimagedata[0];
//            list<int> difs, values;
//            for(int i = 1; i < (int)pcameradata->vimagedata.size()/3; ++i) {
//                curvalue = 0xffffff&*(int*)&pcameradata->vimagedata[3*i];
//                if( curvalue != lastvalue ) {
//                    values.push_back(lastvalue);
//                    difs.push_back(i-lastdiff);
//                    lastdiff = i;
//                    lastvalue = curvalue;
//                }
//            }
//            difs.push_back(pcameradata->vimagedata.size()/3-lastdiff);
//            values.push_back(curvalue);
//
//            sout << values.size() << " ";
//            FOREACH(it, values)
//                sout << *it << " ";
//            sout << difs.size() << " ";
//            FOREACH(it, difs)
//                sout << *it << " ";
        }
        
        return true;
    }

    bool robot_sensorsend_srv(robot_sensorsend::request& req, robot_sensorsend::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        if( req.sensorindex >= probot->GetSensors().size() )
            return false;
        
        RobotBase::AttachedSensor& sensor = probot->GetSensors()[req.sensorindex];
            
        if( sensor.GetSensor() == NULL )
            return false;

        if( !sensor.GetSensor()->SupportsCmd(req.cmd.c_str()) ) {
            RAVELOG_ERRORA("Robot %S sensor %d doesn't support command: \"%s\"\n", probot->GetName(), req.sensorindex, req.cmd.c_str());
            return false;
        }
        
        stringstream ss(req.args);
        stringstream sout;
        bool bsuccess = false;
        AddWorker(new SendCmdSensorWorker(sensor.GetSensor(),ss,sout,bsuccess), true);
        if( bsuccess )
            res.out = sout.str();
        return bsuccess;
    }

    void RobotSetActiveDOFs(RobotBase* probot, openraveros::ActiveDOFs& active)
    {
        vector<int> vjointindices(active.joints.size());
        for(size_t i = 0; i < active.joints.size(); ++i)
            vjointindices[i] = active.joints[i];

        Vector vaxis(active.rotationaxis[0], active.rotationaxis[1], active.rotationaxis[2]);
        LockEnvironment envlock(this);
        probot->SetActiveDOFs(vjointindices, active.affine, (active.affine&RobotBase::DOF_RotationAxis)?&vaxis:NULL);
    }
    
    bool robot_setactivedofs_srv(robot_setactivedofs::request& req, robot_setactivedofs::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        LockEnvironment envlock(this);
        RobotSetActiveDOFs(probot, req.active);
        return true;
    }

    bool robot_setactivevalues_srv(robot_setactivevalues::request& req, robot_setactivevalues::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        if( probot->GetActiveDOF() == 0 )
            return false;

        LockEnvironment envlock(this);
        vector<dReal> vvalues;

        if( req.indices.size() > 0 ) {
            if( req.indices.size() != req.values.size() ) {
                RAVELOG_WARNA("indices (%d) different size than joint values (%d)\n", req.indices.size(), req.values.size());
                return false;
            }

            probot->GetActiveDOFValues(vvalues);
            for(uint32_t i = 0; i < req.indices.size(); ++i)
                vvalues[req.indices[i]] = req.values[i];
        }
        else {
            if( probot->GetActiveDOF() != (int)req.values.size() ) {
                RAVELOG_WARNA("body dof (%d) not equal to values (%d)", probot->GetDOF(), req.values.size());
                return false;
            }

            vvalues.reserve(req.values.size());
            FOREACHC(it,req.values)
                vvalues.push_back(*it);
        }

        probot->SetActiveDOFValues(NULL, &vvalues[0], true);

        if( probot->GetController() != NULL ) {
            probot->GetJointValues(vvalues); // reget the values since they'll go through the joint limits
            probot->GetController()->SetDesired(&vvalues[0]);
        }

        return true;
    }

    bool robot_starttrajectory_srv(robot_starttrajectory::request& req, robot_starttrajectory::response& res)
    {
        KinBody* pbody = GetEnv()->GetBodyFromNetworkId(req.bodyid);
        if( pbody == NULL || !pbody->IsRobot() )
            return false;

        RobotBase* probot = (RobotBase*)pbody;

        LockEnvironment envlock(this);
        RobotBase::RobotStateSaver saver(probot);
        
        RobotSetActiveDOFs(probot, req.trajectory.active);

        boost::shared_ptr<OpenRAVE::Trajectory> pfulltraj(GetEnv()->CreateTrajectory(probot->GetDOF()));
        boost::shared_ptr<OpenRAVE::Trajectory> ptraj(GetEnv()->CreateTrajectory(probot->GetActiveDOF()));
        
        OpenRAVE::Trajectory::TPOINT pt; pt.q.resize(probot->GetActiveDOF());
        pt.trans = probot->GetTransform();

        bool bOverwriteTransforms = !(req.options & robot_starttrajectory::request::Traj_UseTransforms);
        bool bAutoCalcTiming = !(req.options & robot_starttrajectory::request::Traj_UseTimestamps);

        FOREACH(it, req.trajectory.points) {
            ROS_ASSERT( it->position.size() == pt.q.size() );
            for(size_t i = 0; i < pt.q.size(); ++i)
                pt.q[i] = it->position[i];

            pt.qdot.resize(it->velocity.size());
            for(size_t i = 0; i < pt.qdot.size(); ++i)
                pt.qdot[i] = it->velocity[i];

            if( !bOverwriteTransforms )
                pt.trans = FromAffineTransform(it->transform);
            if( !bAutoCalcTiming )
                pt.time = it->timestamp;
        }

        probot->GetFullTrajectoryFromActive(pfulltraj.get(), ptraj.get(), bOverwriteTransforms);
        
        if( !pfulltraj->CalcTrajTiming(probot, pfulltraj->GetInterpMethod(), bAutoCalcTiming, false) )
            return false;

        probot->SetMotion(pfulltraj.get());
        return true;
    }

private:
    AffineTransformMatrix GetAffineTransform(const TransformMatrix& tm)
    {
        AffineTransformMatrix am;
        am.m[0] = tm.m[0]; am.m[3] = tm.m[1]; am.m[6] = tm.m[2];
        am.m[1] = tm.m[4]; am.m[4] = tm.m[5]; am.m[7] = tm.m[6];
        am.m[2] = tm.m[8]; am.m[5] = tm.m[9]; am.m[8] = tm.m[10];
        am.m[9] = tm.trans.x; am.m[10] = tm.trans.y; am.m[11] = tm.trans.z;
        return am;
    }

    TransformMatrix FromAffineTransform(const AffineTransformMatrix& am)
    {
        TransformMatrix tm;
        tm.m[0] = am.m[0]; tm.m[1] = am.m[3]; tm.m[2] = am.m[6];
        tm.m[4] = am.m[1]; tm.m[5] = am.m[4]; tm.m[6] = am.m[7];
        tm.m[8] = am.m[2]; tm.m[9] = am.m[5]; tm.m[10] = am.m[8];
        tm.trans.x = am.m[9]; tm.trans.y = am.m[10]; tm.trans.z = am.m[11];
        return tm;
    }

    AffineTransformMatrix GetAffineIdentity()
    {
        AffineTransformMatrix am;
        am.m[0] = 1; am.m[3] = 0; am.m[6] = 0; am.m[9] = 0;
        am.m[1] = 0; am.m[4] = 1; am.m[7] = 0; am.m[10] = 0;
        am.m[2] = 0; am.m[5] = 0; am.m[8] = 1; am.m[11] = 0;
        return am;
    }

    boost::shared_ptr<SetViewerFunc> _psetviewer;
    boost::shared_ptr<PhysicsEngineBase> _pphysics;
    boost::shared_ptr<CollisionCheckerBase> _pcolchecker;
    map<int, boost::shared_ptr<PlannerBase> > _mapplanners;
    map<int, boost::shared_ptr<ProblemInstance> > _mapproblems;
    map<int, boost::shared_ptr<FIGURE> > _mapFigureIds;
    int _nNextFigureId, _nNextPlannerId, _nNextProblemId;
    float _fSimulationTimestep;
    Vector _vgravity;
    bool _bThreadDestroyed, _bCloseClient;

    /// viewer control variables
    boost::shared_ptr<RaveViewerBase> _pviewer;
    boost::thread _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    
    /// workers
    boost::mutex _mutexWorker;
    list<boost::shared_ptr<WorkExecutor> > _listWorkers;
    boost::condition _conditionWorkers;
};
