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
        SessionState state = getstate(req.sessionid); \
        if( !state._pserver ) \
            return false; \
        state._pserver->srvname##_srv(req,res); \
    }

class SessionServer
{
    class SessionState
    {
    public:
        virtual ~SessionState() {
            _penv->AttachServer(NULL);
            _pserver.reset();
            _penv.reset();
        }

        boost::shared_ptr<ROSServer> _pserver;
        boost::shared_ptr<EnvironmentBase> _penv;
    };

public:
    SessionServer() {
        _pParentEnvironment.reset(CreateEnvironment());
    }
    virtual ~SessionServer() {
        Destroy();
    }

    bool Init() {
        node* pnode = node::instance();
        if( pnode == NULL )
            return false;

        if( !pnode->advertise_service("openrave_session",&SessionServer::session_callback,this) )
            return false;

        // advertise persistent services
        if( !pnode->advertise_service("body_destroy",&SessionServer::body_destroy_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_enable",&SessionServer::body_enable_srv,this,1,true) )
            return false;

        if( !pnode->advertise_service("body_getaabb",&SessionServer::body_getaabb_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_getaabbs",&SessionServer::body_getaabbs_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_getdof",&SessionServer::body_getdof_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_getjointvalues",&SessionServer::body_getjointvalues_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_getlinks",&SessionServer::body_getlinks_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_setjointvalues",&SessionServer::body_setjointvalues_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("body_settransform",&SessionServer::body_settransform_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_checkcollision",&SessionServer::env_checkcollision_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_closefigures",&SessionServer::env_closefigures_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_createbody",&SessionServer::env_createbody_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_createplanner",&SessionServer::env_createplanner_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_createproblem",&SessionServer::env_createproblem_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_createrobot",&SessionServer::env_createrobot_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_destroyproblem",&SessionServer::env_destroyproblem_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_getbodies",&SessionServer::env_getbodies_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_getbody",&SessionServer::env_getbody_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_getrobots",&SessionServer::env_getrobots_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_loadplugin",&SessionServer::env_loadplugin_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_loadscene",&SessionServer::env_loadscene_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_plot",&SessionServer::env_plot_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_raycollision",&SessionServer::env_raycollision_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_set",&SessionServer::env_set_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_triangulate",&SessionServer::env_triangulate_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("env_wait",&SessionServer::env_wait_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("planner_init",&SessionServer::planner_init_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("planner_plan",&SessionServer::planner_plan_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("problem_sendcommand",&SessionServer::problem_sendcommand_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_controllersend",&SessionServer::robot_controllersend_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_controllerset",&SessionServer::robot_controllerset_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_getactivevalues",&SessionServer::robot_getactivevalues_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_sensorgetdata",&SessionServer::robot_sensorgetdata_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_sensorsend",&SessionServer::robot_sensorsend_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_setactivedofs",&SessionServer::robot_setactivedofs_srv,this,1,true) )
            return false;
        if( !pnode->advertise_service("robot_setactivevalues",&SessionServer::robot_setactivevalues_srv,this,1,true) )
            return false;

        return true;
    }

    void Destroy()
    {
        node* pnode = node::instance();
        if( pnode == NULL )
            return;

        pnode->unadvertise_service("openrave_session");
        pnode->unadvertise_service("body_destroy");
        pnode->unadvertise_service("body_enable");
        pnode->unadvertise_service("body_getaabb");
        pnode->unadvertise_service("body_getaabbs");
        pnode->unadvertise_service("body_getdof");
        pnode->unadvertise_service("body_getjointvalues");
        pnode->unadvertise_service("body_getlinks");
        pnode->unadvertise_service("body_setjointvalues");
        pnode->unadvertise_service("body_settransform");
        pnode->unadvertise_service("env_checkcollision");
        pnode->unadvertise_service("env_closefigures");
        pnode->unadvertise_service("env_createbody");
        pnode->unadvertise_service("env_createplanner");
        pnode->unadvertise_service("env_createproblem");
        pnode->unadvertise_service("env_createrobot");
        pnode->unadvertise_service("env_destroyproblem");
        pnode->unadvertise_service("env_getbodies");
        pnode->unadvertise_service("env_getbody");
        pnode->unadvertise_service("env_getrobots");
        pnode->unadvertise_service("env_loadplugin");
        pnode->unadvertise_service("env_loadscene");
        pnode->unadvertise_service("env_plot");
        pnode->unadvertise_service("env_raycollision");
        pnode->unadvertise_service("env_set");
        pnode->unadvertise_service("set_triangulate");
        pnode->unadvertise_service("env_wait");
        pnode->unadvertise_service("planner_init");
        pnode->unadvertise_service("planner_plan");
        pnode->unadvertise_service("problem_sendcommand");
        pnode->unadvertise_service("robot_controllersend");
        pnode->unadvertise_service("robot_controllerset");
        pnode->unadvertise_service("robot_getactivevalues");
        pnode->unadvertise_service("robot_sensorgetdata");
        pnode->unadvertise_service("robot_sensorsend");
        pnode->unadvertise_service("robot_setactivedofs");
        pnode->unadvertise_service("robot_setactivevalues");
    }

private:
    map<int,SessionState> _mapsessions;
    boost::mutex _mutexsession;
    boost::shared_ptr<EnvironmentBase> _pParentEnvironment;

    SessionState getstate(int sessionid)
    {
        boost::mutex::scoped_lock(_mutexsession);
        if( _mapsessions.find(sessionid) == _mapsessions.end() )
            return SessionState();
        return _mapsessions[sessionid];
    }

    bool session_callback(openrave_session::request& req, openrave_session::response& res)
    {
        if( req.sessionid ) {
            // destory the session
            boost::mutex::scoped_lock(_mutexsession);
            if( _mapsessions.find(req.sessionid) != _mapsessions.end() ) {
                _mapsessions.erase(req.sessionid);
                ROS_INFO("destroyed openrave session: %d\n", req.sessionid);
                return true;
            }
            
            return false;
        }

        int id = rand();
        while(id == 0 || _mapsessions.find(id) != _mapsessions.end())
            id = rand();

        SessionState state;
        state._pserver.reset(new ROSServer(state._penv.get()));

        if( req.clone_sessionid ) {
            // clone the environment from clone_sessionid
            SessionState state = getstate(req.clone_sessionid);
            if( !state._penv )
                ROS_INFO("failed to find session %d\n", req.clone_sessionid);
            else 
                state._penv.reset(state._penv->CloneSelf(req.clone_options));
        }

        if( !state._penv ) {
            // cloning from parent
            ROS_DEBUG("cloning from parent\n");
            state._penv.reset(_pParentEnvironment->CloneSelf(0));
        }

        _mapsessions[id] = state;
        res.sessionid = id;

        ROS_INFO("started openrave session: %d\n", id);
        return true;
    }

    REFLECT_SERVICE(body_destroy)
    REFLECT_SERVICE(body_enable)
    REFLECT_SERVICE(body_getaabb)
    REFLECT_SERVICE(body_getaabbs)
    REFLECT_SERVICE(body_getdof)
    REFLECT_SERVICE(body_getjointvalues)
    REFLECT_SERVICE(body_getlinks)
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
