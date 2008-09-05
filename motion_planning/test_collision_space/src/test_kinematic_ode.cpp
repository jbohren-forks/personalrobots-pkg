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

/** \author Ioan Sucan */

/** Test program for displaying and testing ODE collision spaces and
    kinematic models */

#include <collision_space/environmentODE.h>
#include <display_ode/displayODE.h>
#include <algorithm>

using namespace collision_space;
using namespace display_ode;

static DisplayODESpaces                 spaces;
static EnvironmentModelODE             *env   = NULL;
static planning_models::KinematicModel *model = NULL;

static void start(void)
{
    static float xyz[3] = {-0.2179,1.5278,0.8700};
    static float hpr[3] = {-77.5000,-19.5000,0.0000};
    
    dsSetViewpoint(xyz, hpr);
}


static double*     currentParam = NULL;
static std::string currentGroup = "";

static void command(int cmd)
{
    static int direction = 1;
    
    char key = (char)cmd;    
    
    if (key == 'g')
    {
	std::vector<std::string> groups;
	model->getGroups(groups);
	std::vector<std::string>::iterator pos = std::find(groups.begin(), groups.end(), currentGroup);
	if (groups.size() > 0)
	{
	    if (pos == groups.end())
		currentGroup = groups[0];
	    else
	    {
		pos++;
		currentGroup = (pos == groups.end()) ? groups[0] : *pos;
	    }
	}
	for (unsigned int i = 0 ; i < model->stateDimension ; ++i)
	    currentParam[i] = 0.0;
	
	printf("Current group is '%s'\n", currentGroup.c_str());
	
	model->computeTransforms(currentParam, model->getGroupID(currentGroup));
	env->updateRobotModel(0);
    }
    
    if (key == 'd')
	direction = -direction;
    
    if (key >= '0' && key <= '9')
    {
	currentParam[key-'0'] += direction * 0.1;
	
	model->computeTransforms(currentParam, model->getGroupID(currentGroup));
	env->updateRobotModel(0);
    }
        
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
    
    env = new EnvironmentModelODE();
    model = new planning_models::KinematicModel();
    
    model->setVerbose(true);
    model->build(file);
    
    std::vector<planning_models::KinematicModel::Link*> links;
    model->getLinks(links);

    std::vector<std::string> linkNames;
    for (unsigned int i =0 ; i < links.size() ; ++i)
	linkNames.push_back(links[i]->name);
    
    env->addRobotModel(model, linkNames);
    model->printModelInfo();
    
    currentParam = new double[model->stateDimension];

    for (unsigned int i = 0 ; i < model->stateDimension ; ++i)
	currentParam[i] = 0.0;
    model->computeTransforms(currentParam);
    env->updateRobotModel(0);
        
    spaces.addSpace(env->getODESpace(), 1.0f, 0.3f, 0.0f);
    for (unsigned int i = 0 ; i < env->getModelCount() ; ++i)
	spaces.addSpace(env->getModelODESpace(i), 0.0f, 0.5f, (float)(i + 1)/(float)env->getModelCount());
    

    double sphere[3] = {0.8,0.2,0.4};    
    
    env->addPointCloud(1, sphere, 0.1);
    env->setSelfCollision(false);

    printf("\n\nCollision ODE: %d\n", env->isCollision(0));
    
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "./res";
    
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    
    delete env;
    
    delete[] currentParam;
    
    return 0;    
}

