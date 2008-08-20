/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <collision_space/environmentODE.h>
#include <display_ode/displayODE.h>

using namespace collision_space;
using namespace display_ode;

static DisplayODESpaces spaces;

static void start(void)
{
    static float xyz[3] = {-0.2179,1.5278,0.8700};
    static float hpr[3] = {-77.5000,-19.5000,0.0000};
    
    dsSetViewpoint(xyz, hpr);
}

static void command(int cmd)
{
}

static void simLoop(int)
{
    spaces.displaySpaces();
}

int main(int argc, char **argv)
{
    robot_desc::URDF file;
    if (argc > 1)
	file.loadFile(argv[1]);
    else
	file.loadFile("../../robot_descriptions/wg_robot_description/pr2/pr2.xml");
    
    
    EnvironmentModelODE *kmODE = new EnvironmentModelODE();
    planning_models::KinematicModel *model = new planning_models::KinematicModel();
    model->setVerbose(true);
    model->build(file);
    kmODE->addRobotModel(model);
    model->printModelInfo();
    
    double *param = new double[model->stateDimension];    

    for (unsigned int i = 0 ; i < model->stateDimension ; ++i)
	param[i] = 0.0;
    model->computeTransforms(param);
    kmODE->updateRobotModel(0);
    
    param[3] = -M_PI/2.0;    
    param[4] = M_PI/4.0;    
    param[5] = M_PI/4.0;    
    
    
    model->computeTransforms(param, model->getGroupID("pr2::leftArm"));
    kmODE->updateRobotModel(0);

    
    spaces.addSpace(kmODE->getODESpace(), 1.0f, 0.3f, 0.0f);
    for (unsigned int i = 0 ; i < kmODE->getModelCount() ; ++i)
	spaces.addSpace(kmODE->getModelODESpace(i), 0.0f, 0.5f, (float)(i + 1)/(float)kmODE->getModelCount());
    

    double sphere[3] = {0.8,0.2,0.4};    
    
    kmODE->addPointCloud(1, sphere, 0.1);
    kmODE->setSelfCollision(false);

    printf("\n\nCollision ODE: %d\n", kmODE->isCollision(0));
    
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "./res";
    
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    
    delete kmODE;
    
    delete[] param;
    
    return 0;    
}

