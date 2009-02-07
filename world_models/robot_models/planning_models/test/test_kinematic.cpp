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

#include <planning_models/kinematic.h>
#include <gtest/gtest.h>
#include <sstream>

TEST(Loading, EmptyRobot)
{
    static const std::string MODEL0 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"myrobot\">" 
	"</robot>";
    
    planning_models::KinematicModel *model = new planning_models::KinematicModel();
    model->setVerbose(false);
    model->build(MODEL0);
    
    EXPECT_TRUE(model->isBuilt());
    EXPECT_TRUE(model->reduceToRobotFrame());
    
    EXPECT_EQ(std::string("myrobot"), model->getModelName());
    EXPECT_EQ((unsigned int)0, model->getRobotCount());
    EXPECT_EQ((unsigned int)0, model->getModelInfo().stateDimension);
    EXPECT_EQ((unsigned int)0, model->getGroupDimension());
    
    std::vector<planning_models::KinematicModel::Link*> links;
    model->getLinks(links);
    EXPECT_EQ((unsigned int)0, links.size());
    
    std::vector<planning_models::KinematicModel::Joint*> joints;
    model->getJoints(joints);
    EXPECT_EQ((unsigned int)0, joints.size());
    
    std::vector<std::string> groups;
    model->getGroups(groups);    
    EXPECT_EQ((unsigned int)0, groups.size());
       
    delete model;
}


TEST(LoadingAndFK, SimpleRobot)
{
    
    static const std::string MODEL1 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"one_robot\">"
	"<link name=\"base_link\">"
	"  <joint type=\"planar\"/>"
	"  <parent name=\"world\"/>"
	"  <origin rpy=\" 0.0 0 1 \" xyz=\"0.1 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <com xyz=\"0.0 0.099 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision name=\"my_collision\">"
	"    <origin rpy=\"0 0 -1\" xyz=\"-0.1 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"  <map>"
	"    <elem key=\"my_key\" value=\"my_key_val\"/>"
	"  </map>"
	"</link>"
	"<group name=\"base\" flags=\"planning\">"
	"base_link"
	"</group>"
	"</robot>";
    
    static const std::string MODEL1_PARSED = 
	"\n"
	"List of root links in robot 'one_robot' (1) :\n"
	"  Link [base_link]:\n"
	"    - parent link: world\n"
	"    - rpy: (0, 0, 1)\n"
	"    - xyz: (0.1, 0, 0)\n"
	"    Joint [base_link_joint]:\n"
	"      - type: 4\n"
	"      - axis: (0, 0, 0)\n"
	"      - anchor: (0, 0, 0)\n"
	"      - limit: (0, 0)\n"
	"      - effortLimit: 0\n"
	"      - velocityLimit: 0\n"
	"      - calibration: \n"
	"    Collision [my_collision]:\n"
	"      - verbose: No\n"
	"      - rpy: (0, 0, -1)\n"
	"      - xyz: (-0.1, 0, 0)\n"
	"      Geometry [my_collision_geom]:\n"
	"        - type: 2\n"
	"        - size: (1, 2, 1)\n"
	"    Inertial [base_link_inertial]:\n"
	"      - mass: 2.81\n"
	"      - com: (0, 0.099, 0)\n"
	"      - inertia: (0.1, -0.2, 0.5, -0.09, 1, 0.101)\n"
	"    Visual [base_link_visual]:\n"
	"      - rpy: (0, 0, 0)\n"
	"      - xyz: (0, 0, 0)\n"
	"      Geometry [base_link_visual_geom]:\n"
	"        - type: 2\n"
	"        - size: (1, 2, 1)\n"
	"    - groups: base ( planning ) \n"
	"    - children links: \n"
	"    Data flagged as '':\n"
	"      []\n"
	"        my_key = my_key_val\n"
	"\n"
	"Frames:\n"
	"\n"
	"Groups:\n"
	"  Group [base]:\n"
	"    - links: base_link \n"
	"    - frames: \n"
        "    - flags: planning \n"
	"\n"
	"Chains:\n"
	"\n"
	"Data types:\n";

    static const std::string MODEL1_INFO = 
	"Number of robots = 1\n"
	"Complete model state dimension = 3\n"
	"State bounds: [0, 0] [0, 0] [-3.14159, 3.14159] \n"
	"Parameter index:\n"
	"base_link_joint = 0\n"
	"Parameter name:\n"
	"0 = base_link_joint\n"
	"1 = base_link_joint\n"
	"2 = base_link_joint\n"
	"Floating joints at: \n"
	"Planar joints at: 0 base_link_joint \n"
	"Available groups: one_robot::base \n"
	"Group one_robot::base with ID 0 has 1 roots: base_link_joint \n"
	"The state components for this group are: 0 1 2 base_link_joint base_link_joint base_link_joint \n"
	"Link poses:\n"
	"base_link\n"
	"  origin: 9.9, 8, 0\n"
	"  quaternion: 0, 0, -0.479426, 0.877583\n"    
	"\n";
    
    planning_models::KinematicModel *model = new planning_models::KinematicModel();
    model->setVerbose(false);
    robot_desc::URDF *file = new robot_desc::URDF();
    file->loadString(MODEL1.c_str());
    model->build(*file);
    
    EXPECT_TRUE(model->isBuilt());
    EXPECT_TRUE(model->reduceToRobotFrame());
    EXPECT_EQ((unsigned int)3, model->getModelInfo().stateDimension);
    
    std::vector<planning_models::KinematicModel::Joint*> joints;
    model->getJoints(joints);
    EXPECT_EQ((unsigned int)1, joints.size());
    EXPECT_EQ((unsigned int)3, joints[0]->usedParams);

    EXPECT_EQ((unsigned int)1, model->getModelInfo().planarJoints.size());
    EXPECT_EQ(0, model->getModelInfo().planarJoints[0]);

    std::stringstream ssp;
    file->print(ssp);
    EXPECT_EQ(MODEL1_PARSED, ssp.str());
    
    std::stringstream ssi;
    model->printModelInfo(ssi);

    double param[3] = { 10, 8, 0 };
    model->computeTransforms(param);
    
    model->printLinkPoses(ssi);
    EXPECT_EQ(MODEL1_INFO, ssi.str());
    
    delete file;
    delete model;    
}

TEST(FK, OneRobot)
{
    static const std::string MODEL2 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"one_robot\">"
	"<link name=\"base_link\">"
	"  <joint type=\"planar\"/>"
	"  <parent name=\"world\"/>"
	"  <origin rpy=\" 0.0 -0.2 0 \" xyz=\"1 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision name=\"my_collision\">"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_a\">"
	"  <joint type=\"revolute\">"
	"     <axis xyz=\"0 0 1\"/>"
	"  </joint>"
	"  <parent name=\"base_link\"/>"
	"  <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_b\">"
	"  <joint type=\"fixed\"/>"
	"  <parent name=\"link_a\"/>"
	"  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_c\">"
	"  <joint type=\"prismatic\">"
	"    <axis xyz=\"1 0 0\"/>"
	"    <limit max=\"0.396\" min=\"0.0\" safety_length_max=\"0.01\" safety_length_min=\"0.01\"/>"
	"  </joint>"
	"  <parent name=\"link_b\"/>"
	"  <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<group name=\"base\" flags=\"planning\">"
	"base_link "
	"link_a "
	"link_b "
	"link_c "
	"</group>"	
	"</robot>";

    static const std::string MODEL2_INFO = 
	"Number of robots = 1\n"
	"Complete model state dimension = 5\n"
	"State bounds: [0, 0] [0, 0] [-3.14159, 3.14159] [-3.14159, 3.14159] [0.01, 0.386] \n"
	"Parameter index:\n"
	"base_link_joint = 0\n"
	"link_a_joint = 3\n"
	"link_c_joint = 4\n"
	"Parameter name:\n"
	"0 = base_link_joint\n"
	"1 = base_link_joint\n"
	"2 = base_link_joint\n"
	"3 = link_a_joint\n"
	"4 = link_c_joint\n"
	"Floating joints at: \n"
	"Planar joints at: 0 base_link_joint \n"
	"Available groups: one_robot::base \n"
	"Group one_robot::base with ID 0 has 1 roots: base_link_joint \n"
	"The state components for this group are: 0 1 2 3 4 base_link_joint base_link_joint base_link_joint link_a_joint link_c_joint \n"
	"Link poses:\n"
	"base_link\n"
	"  origin: 1, 1, 0\n"
	"  quaternion: 0, 0, 0.247404, 0.968912\n"
	"\n"
	"link_a\n"
	"  origin: 1, 1, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n"
	"link_b\n"
	"  origin: 1, 1.5, 0\n"
	"  quaternion: 0, -0.20846, 0, 0.978031\n"
	"\n"
	"link_c\n"
	"  origin: 1.1, 1.4, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n";
    
    planning_models::KinematicModel *model = new planning_models::KinematicModel();
    model->setVerbose(false);
    model->build(MODEL2);
    
    EXPECT_TRUE(model->isBuilt());
    EXPECT_TRUE(model->reduceToRobotFrame());
    EXPECT_EQ((unsigned int)5, model->getModelInfo().stateDimension);

    double param[5] = { 1, 1, 0.5, -0.5, 0.1 };
    model->computeTransforms(param, model->getGroupID("one_robot::base"));
    
    std::stringstream ss1;
    model->printModelInfo(ss1);
    model->printLinkPoses(ss1);
    
    EXPECT_EQ(-1, model->getGroupID("one_robot"));
    
    model->computeTransforms(param);
    
    std::stringstream ss2;
    model->printModelInfo(ss2);
    model->printLinkPoses(ss2);
    
    EXPECT_EQ(ss1.str(), ss2.str());
    EXPECT_EQ(MODEL2_INFO, ss1.str());
    
    planning_models::KinematicModel::StateParams *sp = model->newStateParams();
    EXPECT_FALSE(sp->seenAll());

    double tmpParam[3];
    tmpParam[0] = 0.1;
    sp->setParams(tmpParam, "link_a_joint");
    EXPECT_FALSE(sp->seenAll());

    tmpParam[0] = -1.0;    
    sp->setParams(tmpParam, "link_c_joint");
    EXPECT_FALSE(sp->seenAll());
    
    tmpParam[0] = 0.5; tmpParam[1] = 0.4; tmpParam[2] = 1.1;
    sp->setParams(tmpParam, "base_link_joint");
    EXPECT_TRUE(sp->seenAll());
    
    EXPECT_EQ(0.5, sp->getParams()[0]);
    EXPECT_EQ(0.4, sp->getParams()[1]);
    EXPECT_EQ(1.1, sp->getParams()[2]);
    EXPECT_EQ(0.1, sp->getParams()[3]);
    EXPECT_EQ(-1.0, sp->getParams()[4]);
    
    delete sp;
    delete model;
}

TEST(FK, MoreRobots)
{
    static const std::string MODEL3 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"more_robots\">"
	"<link name=\"base_link1\">"
	"  <joint type=\"planar\"/>"
	"  <parent name=\"world\"/>"
	"  <origin rpy=\" 0.0 -0.2 0 \" xyz=\"1 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_a\">"
	"  <joint type=\"revolute\">"
	"     <axis xyz=\"0 0 1\"/>"
	"  </joint>"
	"  <parent name=\"base_link1\"/>"
	"  <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_b\">"
	"  <joint type=\"fixed\"/>"
	"  <parent name=\"link_a\"/>"
	"  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_c\">"
	"  <joint type=\"prismatic\">"
	"    <axis xyz=\"1 0 0\"/>"
	"    <limit max=\"0.396\" min=\"0.0\" safety_length_max=\"0.01\" safety_length_min=\"0.01\"/>"
	"  </joint>"
	"  <parent name=\"link_b\"/>"
	"  <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"base_link2\">"
	"  <joint type=\"planar\"/>"
	"  <parent name=\"world\"/>"
	"  <origin rpy=\" 0.0 -0.2 0 \" xyz=\"1 5 0 \"/>"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_d\">"
	"  <joint type=\"revolute\">"
	"     <axis xyz=\"0 0 1\"/>"
	"  </joint>"
	"  <parent name=\"base_link2\"/>"
	"  <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_e\">"
	"  <joint type=\"fixed\"/>"
	"  <parent name=\"link_d\"/>"
	"  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"link_f\">"
	"  <joint type=\"prismatic\">"
	"    <axis xyz=\"1 0 0\"/>"
	"    <limit max=\"0.396\" min=\"0.0\" safety_length_max=\"0.01\" safety_length_min=\"0.01\"/>"
	"  </joint>"
	"  <parent name=\"link_e\"/>"
	"  <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <com xyz=\"0.0 0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<link name=\"base_link3\">"
	"  <joint type=\"planar\"/>"
	"  <parent name=\"world\"/>"
	"  <origin rpy=\" 0.0 -0.2 0 \" xyz=\"1 -5 0 \"/>"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <com xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<group name=\"r1\" flags=\"planning\">"
	"base_link1 "
	"link_a "
	"link_b "
	"link_c "
	"</group>"	
	"<group name=\"r2\" flags=\"planning\">"
	"base_link2 "
	"link_d "
	"link_e "
	"link_f "
	"</group>"	
	"<group name=\"r1r2\" flags=\"planning\">"
	"base_link1 "
	"link_a "
	"link_b "
	"link_c "
	"base_link2 "
	"link_d "
	"link_e "
	"link_f "
	"</group>"
	"<group name=\"parts\" flags=\"planning\">"
	"base_link1 "
	"link_a "
	"link_b "
	"link_e "
	"link_f "
	"base_link3 "
	"</group>"	
	"</robot>";

    static const std::string MODEL3_INFO = 
	"Number of robots = 3\n"
	"Complete model state dimension = 13\n"
	"State bounds: [0, 0] [0, 0] [-3.14159, 3.14159] [-3.14159, 3.14159] [0.01, 0.386] [0, 0] [0, 0] [-3.14159, 3.14159] [-3.14159, 3.14159] [0.01, 0.386] [0, 0] [0, 0] [-3.14159, 3.14159] \n"
	"Parameter index:\n"
	"base_link1_joint = 0\n"
	"base_link2_joint = 5\n"
	"base_link3_joint = 10\n"
	"link_a_joint = 3\n"
	"link_c_joint = 4\n"
	"link_d_joint = 8\n"
	"link_f_joint = 9\n"
	"Parameter name:\n"
	"0 = base_link1_joint\n"
	"1 = base_link1_joint\n"
	"2 = base_link1_joint\n"
	"3 = link_a_joint\n"
	"4 = link_c_joint\n"
	"5 = base_link2_joint\n"
	"6 = base_link2_joint\n"
	"7 = base_link2_joint\n"
	"8 = link_d_joint\n"
	"9 = link_f_joint\n"
	"10 = base_link3_joint\n"
	"11 = base_link3_joint\n"
	"12 = base_link3_joint\n"
	"Floating joints at: \n"
	"Planar joints at: 0 5 10 base_link1_joint base_link2_joint base_link3_joint \n"
	"Available groups: more_robots::parts more_robots::r1 more_robots::r1r2 more_robots::r2 \n"
	"Group more_robots::parts with ID 0 has 3 roots: base_link1_joint link_e_joint base_link3_joint \n"
	"The state components for this group are: 0 1 2 3 9 10 11 12 base_link1_joint base_link1_joint base_link1_joint link_a_joint link_f_joint base_link3_joint base_link3_joint base_link3_joint \n"
	"Group more_robots::r1 with ID 1 has 1 roots: base_link1_joint \n"
	"The state components for this group are: 0 1 2 3 4 base_link1_joint base_link1_joint base_link1_joint link_a_joint link_c_joint \n"
	"Group more_robots::r1r2 with ID 2 has 2 roots: base_link1_joint base_link2_joint \n"
	"The state components for this group are: 0 1 2 3 4 5 6 7 8 9 base_link1_joint base_link1_joint base_link1_joint link_a_joint link_c_joint base_link2_joint base_link2_joint base_link2_joint link_d_joint link_f_joint \n"
	"Group more_robots::r2 with ID 3 has 1 roots: base_link2_joint \n"
	"The state components for this group are: 5 6 7 8 9 base_link2_joint base_link2_joint base_link2_joint link_d_joint link_f_joint \n"
	"Link poses:\n"
	"base_link1\n"
	"  origin: -1, -1, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n"
	"base_link2\n"
	"  origin: 0, 0, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n"
	"base_link3\n"
	"  origin: 5, 5, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n"
	"link_a\n"
	"  origin: -1, -1, 0\n"
	"  quaternion: 0, 0, 0.706825, 0.707388\n"
	"\n"
	"link_b\n"
	"  origin: -1.5, -0.999602, 0\n"
	"  quaternion: 0.147345, -0.147462, 0.691297, 0.691848\n"
	"\n"
	"link_c\n"
	"  origin: -1.39984, -0.801682, 0\n"
	"  quaternion: -1.96183e-17, 0, 0.706825, 0.707388\n"
	"\n"
	"link_d\n"
	"  origin: 0, 0, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n"
	"link_e\n"
	"  origin: 0, 0.5, 0\n"
	"  quaternion: 0, -0.20846, 0, 0.978031\n"
	"\n"
	"link_f\n"
	"  origin: 0, 0.4, 0\n"
	"  quaternion: 0, 0, 0, 1\n"
	"\n";
    
    planning_models::KinematicModel *model = new planning_models::KinematicModel();
    model->setVerbose(false);
    model->build(MODEL3);
    
    EXPECT_TRUE(model->isBuilt());
    EXPECT_TRUE(model->reduceToRobotFrame());
    EXPECT_EQ((unsigned int)13, model->getModelInfo().stateDimension);
    
    model->defaultState();

    std::stringstream ss;
    model->printModelInfo(ss);
    double param[8] = { -1, -1, 0, 1.57, 0.0, 5, 5, 0 };
    model->computeTransforms(param, model->getGroupID("more_robots::parts"));    
    model->printLinkPoses(ss);
    
    EXPECT_EQ(MODEL3_INFO, ss.str());
    delete model;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
