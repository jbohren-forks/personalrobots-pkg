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

/** \Author Ioan Sucan */

#include <collision_space/point_inclusion.h>
#include <gtest/gtest.h>

TEST(SpherePointContainment, SimpleInside)
{
    planning_models::shapes::Sphere shape(1.0);
    collision_space::bodies::Body* sphere = new collision_space::bodies::Sphere(&shape);
    sphere->setScale(1.05);
    bool contains = sphere->containsPoint(0,0,1.0);
    delete sphere;
    EXPECT_TRUE(contains);
}

TEST(SpherePointContainment, SimpleOutside)
{ 
    planning_models::shapes::Sphere shape(1.0);
    collision_space::bodies::Body* sphere = new collision_space::bodies::Sphere(&shape);
    sphere->setScale(0.95);
    bool contains = sphere->containsPoint(0,0,1.0);
    delete sphere;
    EXPECT_FALSE(contains);
}

TEST(SpherePointContainment, ComplexInside)
{ 
    planning_models::shapes::Sphere shape(1.0);
    collision_space::bodies::Body* sphere = new collision_space::bodies::Sphere(&shape);
    sphere->setScale(0.95);
    btTransform pose;
    pose.setIdentity();    
    pose.setOrigin(btVector3(btScalar(1),btScalar(1),btScalar(1)));
    sphere->setPose(pose);
    bool contains = sphere->containsPoint(0.5,1,1.0);
    delete sphere;
    EXPECT_TRUE(contains);
}

TEST(SpherePointContainment, ComplexOutside)
{   
    planning_models::shapes::Sphere shape(1.0);
    collision_space::bodies::Body* sphere = new collision_space::bodies::Sphere(&shape);
    sphere->setScale(0.95);
    btTransform pose;
    pose.setIdentity();    
    pose.setOrigin(btVector3(btScalar(1),btScalar(1),btScalar(1)));
    sphere->setPose(pose);
    bool contains = sphere->containsPoint(0.5,0.0,0.0);
    delete sphere;
    EXPECT_FALSE(contains);
}


TEST(BoxPointContainment, SimpleInside)
{    
    planning_models::shapes::Box shape(1.0, 2.0, 3.0);
    collision_space::bodies::Body* box = new collision_space::bodies::Box(&shape);
    box->setScale(0.95);
    bool contains = box->containsPoint(0,0,1.0);
    delete box;
    EXPECT_TRUE(contains);
}


TEST(BoxPointContainment, SimpleOutside)
{
    planning_models::shapes::Box shape(1.0, 2.0, 3.0);
    collision_space::bodies::Body* box = new collision_space::bodies::Box(&shape);
    box->setScale(0.95);
    bool contains = box->containsPoint(0,0,3.0);
    delete box;
    EXPECT_FALSE(contains);
}


TEST(BoxPointContainment, ComplexInside)
{  
    planning_models::shapes::Box shape(1.0, 1.0, 1.0);
    collision_space::bodies::Body* box = new collision_space::bodies::Box(&shape);
    box->setScale(1.01);
    btTransform pose;
    pose.setIdentity();    
    pose.setOrigin(btVector3(btScalar(1),btScalar(1),btScalar(1)));
    btQuaternion quat(btVector3(btScalar(1), btScalar(0), btScalar(0)), M_PI/3.0);
    pose.setRotation(quat);
    box->setPose(pose);    

    bool contains = box->containsPoint(1.5,1.0,1.5);
    delete box;
    EXPECT_TRUE(contains);
}

TEST(BoxPointContainment, ComplexOutside)
{    
    planning_models::shapes::Box shape(1.0, 1.0, 1.0);
    collision_space::bodies::Body* box = new collision_space::bodies::Box(&shape);
    box->setScale(1.01);
    btTransform pose;
    pose.setIdentity();    
    pose.setOrigin(btVector3(btScalar(1),btScalar(1),btScalar(1)));
    btQuaternion quat(btVector3(btScalar(1), btScalar(0), btScalar(0)), M_PI/3.0);
    pose.setRotation(quat);
    box->setPose(pose);    

    bool contains = box->containsPoint(1.5,1.5,1.5);
    delete box;
    EXPECT_FALSE(contains);
}

TEST(CylinderPointContainment, SimpleInside)
{
    planning_models::shapes::Cylinder shape(1.0, 4.0);
    collision_space::bodies::Body* cylinder = new collision_space::bodies::Cylinder(&shape);
    cylinder->setScale(1.05);
    bool contains = cylinder->containsPoint(0, 0, 2.0);
    delete cylinder;
    EXPECT_TRUE(contains);
}


TEST(CylinderPointContainment, SimpleOutside)
{
    planning_models::shapes::Cylinder shape(1.0, 4.0);
    collision_space::bodies::Body* cylinder = new collision_space::bodies::Cylinder(&shape);
    cylinder->setScale(0.95);
    bool contains = cylinder->containsPoint(0,0,2.0);
    delete cylinder;
    EXPECT_FALSE(contains);
}


int main(int argc, char **argv)
{ 
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
