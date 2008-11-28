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
public:
    ROSServer(boost::shared_ptr<EnvironmentBase> penv) : RaveServerBase(penv.get()), _nNextFigureId(1), _nNextPlannerId(1) {
        _penv = penv;
        _bThreadDestroyed = false;
        _fSimulationTimestep = 0.01;
        _vgravity = Vector(0,0,-9.8f);
    }
    virtual ~ROSServer() {
        Destroy();
    }
    
    virtual void Destroy()
    {
        Reset();
        
        _penv->SetCollisionChecker(NULL);
        _penv->SetPhysicsEngine(NULL);
        _penv->AttachViewer(NULL);
        
        // have to maintain a certain destruction order
        _pphysics.reset();
        _pcolchecker.reset();

        if( !!_pviewer ) {
            _pviewer->quitmainloop();
            _threadviewer.join();
            _pviewer.reset();
        }
    }

    virtual void Reset()
    {
                
//        pthread_mutex_lock(&_mutexWorker);
//        listWorkers.clear();
//        pthread_mutex_unlock(&_mutexWorker);
//        pthread_cond_signal(&_condWorker);
//
//        FOREACH(it, s_mapFigureIds)  
//            g_Environ.closegraph(it->second);
//        s_mapFigureIds.clear();

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
    }

    virtual bool IsInit()
    {
        return true;
    }

    virtual bool IsClosing()
    {
        return false;
    }

    // called from threads other than the main worker to wait until
    virtual void SyncWithWorkerThread()
    {
    }

    bool body_destroy_srv(body_destroy::request& req, body_destroy::response& res);
    bool body_enable_srv(body_enable::request& req, body_enable::response& res);
    bool body_getaabb_srv(body_getaabb::request& req, body_getaabb::response& res);
    bool body_getaabbs_srv(body_getaabbs::request& req, body_getaabbs::response& res);
    bool body_getdof_srv(body_getdof::request& req, body_getdof::response& res);
    bool body_getlinks_srv(body_getlinks::request& req, body_getlinks::response& res);
    bool body_setjointvalues_srv(body_setjointvalues::request& req, body_setjointvalues::response& res);
    bool body_settransform_srv(body_settransform::request& req, body_settransform::response& res);
    bool env_checkcollision_srv(env_checkcollision::request& req, env_checkcollision::response& res);
    bool env_closefigures_srv(env_closefigures::request& req, env_closefigures::response& res);
    bool env_createbody_srv(env_createbody::request& req, env_createbody::response& res);
    bool env_createplanner_srv(env_createplanner::request& req, env_createplanner::response& res);
    bool env_createproblem_srv(env_createproblem::request& req, env_createproblem::response& res);
    bool env_createrobot_srv(env_createrobot::request& req, env_createrobot::response& res);
    bool env_destroyproblem_srv(env_destroyproblem::request& req, env_destroyproblem::response& res);
    bool env_getbodies_srv(env_getbodies::request& req, env_getbodies::response& res);
    bool env_getbody_srv(env_getbody::request& req, env_getbody::response& res);
    bool env_getrobots_srv(env_getrobots::request& req, env_getrobots::response& res);
    bool env_loadplugin_srv(env_loadplugin::request& req, env_loadplugin::response& res);
    bool env_loadscene_srv(env_loadscene::request& req, env_loadscene::response& res);
    bool env_plot_srv(env_plot::request& req, env_plot::response& res);
    bool env_raycollision_srv(env_raycollision::request& req, env_raycollision::response& res);
    bool env_set_srv(env_set::request& req, env_set::response& res)
    {
        if( req.setmask & env_set::request::Set_Simulation ) {
            switch(req.sim_action) {
            case env_set::request::SimAction_Start:
                _penv->StartSimulation(_fSimulationTimestep);
                break;
            case env_set::request::SimAction_Stop:
                _penv->StopSimulation();
                break;
            case env_set::request::SimAction_Timestep:
                _fSimulationTimestep = req.sim_timestep;
                _penv->StartSimulation(_fSimulationTimestep);
                break;
            }
        }
        if( req.setmask & env_set::request::Set_PhysicsEngine ) {
            _pphysics.reset(_penv->CreatePhysicsEngine(req.physicsengine.c_str()));
            _penv->SetPhysicsEngine(_pphysics.get());
            if( !!_pphysics )
                _pphysics->SetGravity(_vgravity);
        }
        if( req.setmask & env_set::request::Set_CollisionChecker ) {
            int options = _penv->GetCollisionOptions();
            _pcolchecker.reset(_penv->CreateCollisionChecker(req.collisionchecker.c_str()));
            _penv->SetCollisionChecker(_pcolchecker.get());
            _penv->SetCollisionOptions(options);
        }
        if( req.setmask & env_set::request::Set_Gravity ) {
            _vgravity = Vector(req.gravity[0],req.gravity[1],req.gravity[2]);
            if( !!_pphysics )
                _pphysics->SetGravity(_vgravity);
        }
        if( req.setmask & env_set::request::Set_PublishAnytime ) {
            _penv->SetPublishBodiesAnytime(req.publishanytime>0);
        }
        if( req.setmask & env_set::request::Set_DebugLevel ) {
            _penv->SetDebugLevel(req.debuglevel);
        }
        if( req.setmask & env_set::request::Set_Viewer ) {
            if( !!_pviewer ) {
                _pviewer->quitmainloop();
                // no need to wait for joins
                //_threadviewer.join();
            }

            _pviewer.reset(_penv->CreateViewer(req.viewer.c_str()));
            _penv->AttachViewer(_pviewer.get());
            if( !!_pviewer )
                _threadviewer = boost::thread(boost::bind(&RaveViewerBase::main, _pviewer.get()));
        }

        return true;
    }

    bool env_triangulate_srv(env_triangulate::request& req, env_triangulate::response& res);
    bool env_wait_srv(env_wait::request& req, env_wait::response& res);
    bool planner_init_srv(planner_init::request& req, planner_init::response& res);
    bool planner_plan_srv(planner_plan::request& req, planner_plan::response& res);
    bool problem_sendcommand_srv(problem_sendcommand::request& req, problem_sendcommand::response& res);
    bool robot_controllersend_srv(robot_controllersend::request& req, robot_controllersend::response& res);
    bool robot_controllerset_srv(robot_controllerset::request& req, robot_controllerset::response& res);
    bool robot_getactivedof_srv(robot_getactivedof::request& req, robot_getactivedof::response& res);
    bool robot_getactivevalues_srv(robot_getactivevalues::request& req, robot_getactivevalues::response& res);
    bool robot_sensorgetdata_srv(robot_sensorgetdata::request& req, robot_sensorgetdata::response& res);
    bool robot_sensorsend_srv(robot_sensorsend::request& req, robot_sensorsend::response& res);
    bool robot_setactivedofs_srv(robot_setactivedofs::request& req, robot_setactivedofs::response& res);
    bool robot_setactivevalues_srv(robot_setactivevalues::request& req, robot_setactivevalues::response& res);
    bool robot_starttrajectory_srv(robot_starttrajectory::request& req, robot_starttrajectory::response& res);

private:
    boost::shared_ptr<EnvironmentBase> _penv; ///< the environment this instance of this session
    boost::shared_ptr<PhysicsEngineBase> _pphysics;
    boost::shared_ptr<CollisionCheckerBase> _pcolchecker;
    boost::shared_ptr<RaveViewerBase> _pviewer;
    map<int, boost::shared_ptr<PlannerBase> > _mapplanners;
    map<int, boost::shared_ptr<ProblemInstance> > _mapproblems;
    map<int, void*> _mapFigureIds;
    int _nNextFigureId, _nNextPlannerId;
    boost::thread _threadviewer;
    bool _bThreadDestroyed;
    float _fSimulationTimestep;
    Vector _vgravity;
};
