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

void printModelInfo(planning_models::KinematicModel *m)
{   
    printf("Number of robots = %d\n", m->getRobotCount());
    printf("Complete model state dimension = %d\n", m->stateDimension);

    printf("State bounds: ");
    for (unsigned int i = 0 ; i < m->stateDimension ; ++i)
	printf("[%f, %f] ", m->stateBounds[2 * i], m->stateBounds[2 * i + 1]);
    printf("\n");
    
    printf("Floating joints at: ");    
    for (unsigned int i = 0 ; i < m->floatingJoints.size() ; ++i)
	printf("%d ", m->floatingJoints[i]);
    printf("\n");

    printf("Available groups: ");    
    std::vector<std::string> l;    
    m->getGroups(l);
    for (unsigned int i = 0 ; i < l.size() ; ++i)
	printf("%s ", l[i].c_str());
    printf("\n");
    
    for (unsigned int i = 0 ; i < l.size() ; ++i)
    {
	int gid = m->getGroupID(l[i]);
	printf("Group %s has %d roots\n", l[i].c_str(), m->groupChainStart[gid].size());
	printf("The state components for this group are: ");
	for (unsigned int j = 0 ; j < m->groupStateIndexList[gid].size() ; ++j)
	    printf("%d ", m->groupStateIndexList[gid][j]);
	printf("\n");
    }    
}

int main(int argc, char **argv)
{
    robot_desc::URDF model;
    model.loadFile("/u/isucan/ros/ros-pkg/robot_descriptions/wg_robot_description/pr2/pr2.xml");
    
    EnvironmentModel *km = new EnvironmentModelODE();
    km->addRobotModel(model);    
    planning_models::KinematicModel *m = km->getModel(0);
    printModelInfo(m);
    
    double *param = new double[m->stateDimension];    

    for (unsigned int i = 0 ; i < m->stateDimension ; ++i)
	param[i] = 0.0;    
    m->computeTransforms(param);
    km->updateRobotModel(0);
    

    for (unsigned int i = 0 ; i < m->stateDimension ; ++i)
	param[i] = 0.1;
    m->computeTransforms(param, m->getGroupID("pr2::leftArm"));
    km->updateRobotModel(0);
    
    EnvironmentModelODE* okm = dynamic_cast<EnvironmentModelODE*>(km);
    spaces.addSpace(okm->getODESpace(), 1.0f, 0.3f, 0.0f);
    for (unsigned int i = 0 ; i < okm->getModelCount() ; ++i)
	spaces.addSpace(okm->getModelODESpace(i), 0.0f, 0.5f, (float)(i + 1)/(float)okm->getModelCount());
    
    double sphere[3] = {0.8,0.2,0.4};    
    
    km->addPointCloud(1, sphere, 0.15);
    km->setSelfCollision(false);
    
    printf("Collision: %d\n", km->isCollision(0));
    

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "./res";
    
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    
    delete km;
    
    delete[] param;
    
    return 0;    
}

