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

#include <openraveros/openrave_session.h>

#include <openraveros/body_destroy.h>
#include <openraveros/body_enable.h>
#include <openraveros/body_getaabb.h>
#include <openraveros/body_getaabbs.h>
#include <openraveros/body_getdof.h>
#include <openraveros/body_getlinks.h>
#include <openraveros/body_setjointvalues.h>
#include <openraveros/body_settransform.h>
#include <openraveros/env_checkcollision.h>
#include <openraveros/env_closefigures.h>
#include <openraveros/env_createbody.h>
#include <openraveros/env_createplanner.h>
#include <openraveros/env_createproblem.h>
#include <openraveros/env_createrobot.h>
#include <openraveros/env_destroyproblem.h>
#include <openraveros/env_getbodies.h>
#include <openraveros/env_getbody.h>
#include <openraveros/env_getrobots.h>
#include <openraveros/env_loadplugin.h>
#include <openraveros/env_loadscene.h>
#include <openraveros/env_plot.h>
#include <openraveros/env_raycollision.h>
#include <openraveros/env_set.h>
#include <openraveros/env_triangulate.h>
#include <openraveros/env_wait.h>
#include <openraveros/planner_init.h>
#include <openraveros/planner_plan.h>
#include <openraveros/problem_sendcommand.h>
#include <openraveros/robot_controllersend.h>
#include <openraveros/robot_controllerset.h>
#include <openraveros/robot_getactivedof.h>
#include <openraveros/robot_getactivevalues.h>
#include <openraveros/robot_sensorgetdata.h>
#include <openraveros/robot_sensorsend.h>
#include <openraveros/robot_setactivedofs.h>
#include <openraveros/robot_setactivevalues.h>
using namespace openraveros;

class ROSServer : public RaveServerBase
{
public:
    ROSServer();
    virtual ~ROSServer();

    virtual void Destroy();
    virtual void Reset();

    virtual bool Init(int port);

    /// worker thread called from the main environment thread
    virtual void Worker();

    virtual bool IsInit();
    virtual bool IsClosing();

    // called from threads other than the main worker to wait until
    virtual void SyncWithWorkerThread();

private:
    bool session_callback(openrave_session::request& openrave_session::response& res);

    bool body_destroy_srv(body_destroy::request& body_destroy::response& res);
    bool body_enable_srv(body_enable::request& body_enable::response& res);
    bool body_getaabb_srv(body_getaabb::request& body_getaabb::response& res);
    bool body_getaabbs_srv(body_getaabbs::request& body_getaabbs::response& res);
    bool body_getdof_srv(body_getdof::request& body_getdof::response& res);
    bool body_getlinks_srv(body_getlinks::request& body_getlinks::response& res);
    bool body_setjointvalues_srv(body_setjointvalues::request& body_setjointvalues::response& res);
    bool body_settransform_srv(body_settransform::request& body_settransform::response& res);
    bool env_checkcollision_srv(env_checkcollision::request& env_checkcollision::response& res);
    bool env_closefigures_srv(env_closefigures::request& env_closefigures::response& res);
    bool env_createbody_srv(env_createbody::request& env_createbody::response& res);
    bool env_createplanner_srv(env_createplanner::request& env_createplanner::response& res);
    bool env_createproblem_srv(env_createproblem::request& env_createproblem::response& res);
    bool env_createrobot_srv(env_createrobot::request& env_createrobot::response& res);
    bool env_destroyproblem_srv(env_destroyproblem::request& env_destroyproblem::response& res);
    bool env_getbodies_srv(env_getbodies::request& env_getbodies::response& res);
    bool env_getbody_srv(env_getbody::request& env_getbody::response& res);
    bool env_getrobots_srv(env_getrobots::request& env_getrobots::response& res);
    bool env_loadplugin_srv(env_loadplugin::request& env_loadplugin::response& res);
    bool env_loadscene_srv(env_loadscene::request& env_loadscene::response& res);
    bool env_plot_srv(env_plot::request& env_plot::response& res);
    bool env_raycollision_srv(env_raycollision::request& env_raycollision::response& res);
    bool env_set_srv(env_set::request& env_set::response& res);
    bool env_triangulate_srv(env_triangulate::request& env_triangulate::response& res);
    bool env_wait_srv(env_wait::request& env_wait::response& res);
    bool planner_init_srv(planner_init::request& planner_init::response& res);
    bool planner_plan_srv(planner_plan::request& planner_plan::response& res);
    bool problem_sendcommand_srv(problem_sendcommand::request& problem_sendcommand::response& res);
    bool robot_controllersend_srv(robot_controllersend::request& robot_controllersend::response& res);
    bool robot_controllerset_srv(robot_controllerset::request& robot_controllerset::response& res);
    bool robot_getactivedof_srv(robot_getactivedof::request& robot_getactivedof::response& res);
    bool robot_getactivevalues_srv(robot_getactivevalues::request& robot_getactivevalues::response& res);
    bool robot_sensorgetdata_srv(robot_sensorgetdata::request& robot_sensorgetdata::response& res);
    bool robot_sensorsend_srv(robot_sensorsend::request& robot_sensorsend::response& res);
    bool robot_setactivedofs_srv(robot_setactivedofs::request& robot_setactivedofs::response& res);
    bool robot_setactivevalues_srv(robot_setactivevalues::request& robot_setactivevalues::response& res);
};
