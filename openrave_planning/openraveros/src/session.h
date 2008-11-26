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
#include <openraveros/openrave_session.h>
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
    bool
};
