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
#include "rosserver.h"

#include <openraveros/openrave_session.h>

using namespace ros;

#define REFLECT_SERVICE(srvname) \
    bool srvname##_srv(srvname::request& req, srvname::response& res) \
    { \
        SessionState state = getstate(req); /* need separate copy in order to guarantee thread safety */ \
        if( !state._pserver ) { \
            ROS_INFO("failed to find session for service %s\n", #srvname); \
            return false; \
        } \
        return state._pserver->srvname##_srv(req,res); \
    }

class SessionServer
{
    class SessionState
    {
    public:
        virtual ~SessionState() {
            _pserver.reset();
            _penv.reset();
        }

        boost::shared_ptr<ROSServer> _pserver;
        boost::shared_ptr<EnvironmentBase> _penv;
    };

    class SessionSetViewerFunc : public SetViewerFunc
    {
    public:
        SessionSetViewerFunc(SessionServer* pserv) : _pserv(pserv) {}
        virtual bool SetViewer(EnvironmentBase* penv, const string& viewername) {
            return _pserv->SetViewer(penv,viewername);
        }

    private:
        SessionServer* _pserv;
    };

    string _sessionname;
public:
    SessionServer() : _penvViewer(NULL), _ok(true) {
        _pParentEnvironment.reset(CreateEnvironment());
    }
    virtual ~SessionServer() {
        Destroy();
    }

    bool Init()
    {
        node* pnode = node::instance();
        if( pnode == NULL )
            return false;
        
        if( !pnode->advertiseService("openrave_session",&SessionServer::session_callback,this, 1) )
            return false;
        _sessionname = pnode->mapName("openrave_session");

        // advertise persistent services
        if( !pnode->advertiseService("body_destroy",&SessionServer::body_destroy_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_enable",&SessionServer::body_enable_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_getaabb",&SessionServer::body_getaabb_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_getaabbs",&SessionServer::body_getaabbs_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_getdof",&SessionServer::body_getdof_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_getjointvalues",&SessionServer::body_getjointvalues_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_setjointvalues",&SessionServer::body_setjointvalues_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("body_settransform",&SessionServer::body_settransform_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_checkcollision",&SessionServer::env_checkcollision_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_closefigures",&SessionServer::env_closefigures_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_createbody",&SessionServer::env_createbody_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_createplanner",&SessionServer::env_createplanner_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_createproblem",&SessionServer::env_createproblem_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_createrobot",&SessionServer::env_createrobot_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_destroyproblem",&SessionServer::env_destroyproblem_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_getbodies",&SessionServer::env_getbodies_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_getbody",&SessionServer::env_getbody_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_getrobots",&SessionServer::env_getrobots_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_loadplugin",&SessionServer::env_loadplugin_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_loadscene",&SessionServer::env_loadscene_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_plot",&SessionServer::env_plot_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_raycollision",&SessionServer::env_raycollision_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_set",&SessionServer::env_set_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_triangulate",&SessionServer::env_triangulate_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("env_wait",&SessionServer::env_wait_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("planner_init",&SessionServer::planner_init_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("planner_plan",&SessionServer::planner_plan_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("problem_sendcommand",&SessionServer::problem_sendcommand_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_controllersend",&SessionServer::robot_controllersend_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_controllerset",&SessionServer::robot_controllerset_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_getactivevalues",&SessionServer::robot_getactivevalues_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_sensorgetdata",&SessionServer::robot_sensorgetdata_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_sensorsend",&SessionServer::robot_sensorsend_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_setactivedofs",&SessionServer::robot_setactivedofs_srv,this,-1) )
            return false;
        if( !pnode->advertiseService("robot_setactivevalues",&SessionServer::robot_setactivevalues_srv,this,-1) )
            return false;

        _ok = true;
        _threadviewer = boost::thread(boost::bind(&SessionServer::ViewerThread, this));

        return true;
    }

    void Destroy()
    {
        selfDestruct();

        node* pnode = node::instance();
        if( pnode == NULL )
            return;

        pnode->unadvertiseService("openrave_session");
        pnode->unadvertiseService("body_destroy");
        pnode->unadvertiseService("body_enable");
        pnode->unadvertiseService("body_getaabb");
        pnode->unadvertiseService("body_getaabbs");
        pnode->unadvertiseService("body_getdof");
        pnode->unadvertiseService("body_getjointvalues");
        pnode->unadvertiseService("body_setjointvalues");
        pnode->unadvertiseService("body_settransform");
        pnode->unadvertiseService("env_checkcollision");
        pnode->unadvertiseService("env_closefigures");
        pnode->unadvertiseService("env_createbody");
        pnode->unadvertiseService("env_createplanner");
        pnode->unadvertiseService("env_createproblem");
        pnode->unadvertiseService("env_createrobot");
        pnode->unadvertiseService("env_destroyproblem");
        pnode->unadvertiseService("env_getbodies");
        pnode->unadvertiseService("env_getbody");
        pnode->unadvertiseService("env_getrobots");
        pnode->unadvertiseService("env_loadplugin");
        pnode->unadvertiseService("env_loadscene");
        pnode->unadvertiseService("env_plot");
        pnode->unadvertiseService("env_raycollision");
        pnode->unadvertiseService("env_set");
        pnode->unadvertiseService("set_triangulate");
        pnode->unadvertiseService("env_wait");
        pnode->unadvertiseService("planner_init");
        pnode->unadvertiseService("planner_plan");
        pnode->unadvertiseService("problem_sendcommand");
        pnode->unadvertiseService("robot_controllersend");
        pnode->unadvertiseService("robot_controllerset");
        pnode->unadvertiseService("robot_getactivevalues");
        pnode->unadvertiseService("robot_sensorgetdata");
        pnode->unadvertiseService("robot_sensorsend");
        pnode->unadvertiseService("robot_setactivedofs");
        pnode->unadvertiseService("robot_setactivevalues");

        _threadviewer.join();
    }
    
    virtual void spin()
    {
        while (_ok) {
            usleep(1000);
        };
    }

    virtual void selfDestruct()
    {
        _ok = false;
        boost::mutex::scoped_lock lockcreate(_mutexViewer);
        if( !!_penvViewer )
            _penvViewer->AttachViewer(NULL);
    }

    bool SetViewer(EnvironmentBase* penv, const string& viewer)
    {
        boost::mutex::scoped_lock lock(_mutexViewer);
        
        // destroy the old viewer
        if( !!_penvViewer ) {
            _penvViewer->AttachViewer(NULL);

            _conditionViewer.wait(lock);
        }
         
        ROS_ASSERT(!_penvViewer);

        _strviewer = viewer;
        if( viewer.size() == 0 || !penv )
            return false;
            
        _penvViewer = penv;
        _conditionViewer.wait(lock);
        return !!_pviewer;
    }

private:
    map<int,SessionState> _mapsessions;
    boost::mutex _mutexsession;
    boost::shared_ptr<EnvironmentBase> _pParentEnvironment;

    // persistent viewer
    boost::shared_ptr<RaveViewerBase> _pviewer;
    boost::thread _threadviewer; ///< persistent thread (qtcoin openrave plugin needs this)
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    EnvironmentBase* _penvViewer;
    string _strviewer;

    bool _ok;

    virtual void ViewerThread()
    {
        while(_ok) {
            
            {
                boost::mutex::scoped_lock lockcreate(_mutexViewer);
                if( _strviewer.size() == 0 || !_penvViewer ) {
                    usleep(1000);
                    continue;
                }

                _pviewer.reset(_penvViewer->CreateViewer(_strviewer.c_str()));
                if( !!_pviewer ) {
                    _penvViewer->AttachViewer(_pviewer.get());
                    _pviewer->ViewerSetSize(1024,768);
                    usleep(100000); // give it some time to initialize
                }

                if( !_pviewer )
                    _penvViewer = NULL;

                _conditionViewer.notify_all();

                if( !_pviewer )
                    continue;
            }

            _pviewer->main(); // spin until quitfrommainloop is called
            
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            RAVELOG_DEBUGA("destroying viewer\n");
            ROS_ASSERT(_penvViewer == _pviewer->GetEnv());
            _penvViewer->AttachViewer(NULL);
            _pviewer.reset();
            _penvViewer = NULL;
            usleep(200000); // give some time for destruction
            _conditionViewer.notify_all();
        }
    }

    template <class MReq>
    SessionState getstate(const MReq& req)
    {
        if( !req.__service_header )
            return SessionState();

        ros::MSGHEADERMAP::const_iterator it = req.__service_header->find(_sessionname);
        if( it == req.__service_header->end() )
            return SessionState();

        boost::mutex::scoped_lock lock(_mutexsession);
        
        int sessionid = atoi(it->second.c_str());
        if( _mapsessions.find(sessionid) == _mapsessions.end() )
            return SessionState();
        return _mapsessions[sessionid];
    }

    bool session_callback(openrave_session::request& req, openrave_session::response& res)
    {
        if( req.sessionid != 0 ) {
            // destory the session
            boost::mutex::scoped_lock lock(_mutexsession);
            if( _mapsessions.find(req.sessionid) != _mapsessions.end() ) {
                _mapsessions.erase(req.sessionid);
                ROS_INFO("destroyed openrave session: %d", req.sessionid);
                return true;
            }
            
            return false;
        }

        int id = rand();
        while(id == 0 || _mapsessions.find(id) != _mapsessions.end())
            id = rand();

        SessionState state;
        
        if( req.clone_sessionid ) {
            // clone the environment from clone_sessionid
            SessionState clonestate;
            {
                boost::mutex::scoped_lock lock(_mutexsession);
                clonestate = _mapsessions[req.clone_sessionid];
            }

            if( !clonestate._penv )
                ROS_INFO("failed to find session %d", req.clone_sessionid);
            else 
                state._penv.reset(clonestate._penv->CloneSelf(req.clone_options));
        }

        if( !state._penv ) {
            // cloning from parent
            ROS_DEBUG("cloning from parent");
            state._penv.reset(_pParentEnvironment->CloneSelf(0));
        }

        state._pserver.reset(new ROSServer(new SessionSetViewerFunc(this), state._penv.get(), req.physicsengine, req.collisionchecker, req.viewer));
        state._penv->AttachServer(state._pserver.get());
        _mapsessions[id] = state;
        res.sessionid = id;

        ROS_INFO("started openrave session: %d, total: %d", id, _mapsessions.size());
        return true;
    }

    REFLECT_SERVICE(body_destroy)
    REFLECT_SERVICE(body_enable)
    REFLECT_SERVICE(body_getaabb)
    REFLECT_SERVICE(body_getaabbs)
    REFLECT_SERVICE(body_getdof)
    REFLECT_SERVICE(body_getjointvalues)
    REFLECT_SERVICE(body_setjointvalues)
    REFLECT_SERVICE(body_settransform)
    REFLECT_SERVICE(env_checkcollision)
    REFLECT_SERVICE(env_closefigures)
    REFLECT_SERVICE(env_createbody)
    REFLECT_SERVICE(env_createplanner)
    REFLECT_SERVICE(env_createproblem)
    REFLECT_SERVICE(env_createrobot)
    REFLECT_SERVICE(env_destroyproblem)
    REFLECT_SERVICE(env_getbodies)
    REFLECT_SERVICE(env_getbody)
    REFLECT_SERVICE(env_getrobots)
    REFLECT_SERVICE(env_loadplugin)
    REFLECT_SERVICE(env_loadscene)
    REFLECT_SERVICE(env_plot)
    REFLECT_SERVICE(env_raycollision)
    REFLECT_SERVICE(env_set)
    REFLECT_SERVICE(env_triangulate)
    REFLECT_SERVICE(env_wait)
    REFLECT_SERVICE(planner_init)
    REFLECT_SERVICE(planner_plan)
    REFLECT_SERVICE(problem_sendcommand)
    REFLECT_SERVICE(robot_controllersend)
    REFLECT_SERVICE(robot_controllerset)
    REFLECT_SERVICE(robot_getactivevalues)
    REFLECT_SERVICE(robot_sensorgetdata)
    REFLECT_SERVICE(robot_sensorsend)
    REFLECT_SERVICE(robot_setactivedofs)
    REFLECT_SERVICE(robot_setactivevalues)
    REFLECT_SERVICE(robot_starttrajectory)
};

// check that message constants match OpenRAVE constants
BOOST_STATIC_ASSERT(EnvironmentBase::Clone_Bodies==openrave_session::request::CloneBodies);
BOOST_STATIC_ASSERT(EnvironmentBase::Clone_Viewer==openrave_session::request::CloneViewer);
BOOST_STATIC_ASSERT(EnvironmentBase::Clone_Simulation==openrave_session::request::CloneSimulation);
BOOST_STATIC_ASSERT(EnvironmentBase::Clone_RealControllers==openrave_session::request::CloneRealControllers);

BOOST_STATIC_ASSERT(ActiveDOFs::DOF_X==RobotBase::DOF_X);
BOOST_STATIC_ASSERT(ActiveDOFs::DOF_Y==RobotBase::DOF_Y);
BOOST_STATIC_ASSERT(ActiveDOFs::DOF_Z==RobotBase::DOF_Z);
BOOST_STATIC_ASSERT(ActiveDOFs::DOF_RotationAxis==RobotBase::DOF_RotationAxis);
BOOST_STATIC_ASSERT(ActiveDOFs::DOF_Rotation3D==RobotBase::DOF_Rotation3D);
BOOST_STATIC_ASSERT(ActiveDOFs::DOF_RotationQuat==RobotBase::DOF_RotationQuat);

BOOST_STATIC_ASSERT(env_checkcollision::request::CO_Distance==CO_Distance);
BOOST_STATIC_ASSERT(env_checkcollision::request::CO_UseTolerance==CO_UseTolerance);
BOOST_STATIC_ASSERT(env_checkcollision::request::CO_Contacts==CO_Contacts);
BOOST_STATIC_ASSERT(env_checkcollision::request::CO_RayAnyHit==CO_RayAnyHit);
