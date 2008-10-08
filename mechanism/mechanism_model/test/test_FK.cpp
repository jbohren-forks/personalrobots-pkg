/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <vector>
#include "mechanism_model/robot.h"

using namespace mechanism;

class LinkageTest : public testing::Test
{
protected:
  LinkageTest()
    : link_models(4), links(4),
      joint_models(3), joints(3)
  {
  }

  virtual void SetUp()
  {
    unsigned int i;
    for (i = 0; i < links.size(); ++i)
      links[i].link_ = &link_models[i];
    for (i = 0; i < joints.size(); ++i) {
      joints[i].joint_ = &joint_models[i];
      joint_models[i].type_ = JOINT_CONTINUOUS;
    }

    // Link 0
    // Stationary
    // Link 1
    link_models[1].origin_xyz_.setValue(1, 0, 0);
    joint_models[0].axis_.setValue(0, 0, 1);
    // Link 2
    link_models[2].origin_xyz_.setValue(1, 0, 0);
    joint_models[1].axis_.setValue(0, 1, 0);
    // Link 3
    link_models[3].origin_xyz_.setValue(1, 0, 0);
    joint_models[2].axis_.setValue(0, 0, 1);
  }

  virtual void TearDown()
  {
  }

  void propagate()
  {
    links[0].propagateFK(NULL, NULL);
    for (unsigned int i = 1; i < links.size(); ++i)
      links[i].propagateFK(&links[i-1], &joints[i-1]);
  }

  void set_joint(int i, double p, double v)
  {
    joints[i].position_ = p;
    joints[i].velocity_ = v;
  }

  std::vector<Link> link_models;
  std::vector<LinkState> links;
  std::vector<Joint> joint_models;
  std::vector<JointState> joints;
};

TEST_F(LinkageTest, AtZeroPosition)
{
  set_joint(0, 0, 0);
  set_joint(1, 0, 0);
  set_joint(2, 0, 0);

  propagate();

  EXPECT_NEAR(links[1].abs_position_[0], 1, 0.001);
  EXPECT_NEAR(links[1].abs_position_[1], 0, 0.001);
  EXPECT_NEAR(links[1].abs_position_[2], 0, 0.001);
  EXPECT_NEAR(links[2].abs_position_[0], 2, 0.001);
  EXPECT_NEAR(links[2].abs_position_[1], 0, 0.001);
  EXPECT_NEAR(links[2].abs_position_[2], 0, 0.001);
  EXPECT_NEAR(links[3].abs_position_[0], 3, 0.001);
  EXPECT_NEAR(links[3].abs_position_[1], 0, 0.001);
  EXPECT_NEAR(links[3].abs_position_[2], 0, 0.001);
}

TEST_F(LinkageTest, Joint0Bent)
{
  set_joint(0, M_PI/2.0, 0);
  set_joint(1, 0, 0);
  set_joint(2, 0, 0);

  propagate();

  EXPECT_NEAR(links[1].abs_position_[0], 1, 0.001);
  EXPECT_NEAR(links[1].abs_position_[1], 0, 0.001);
  EXPECT_NEAR(links[1].abs_position_[2], 0, 0.001);

  EXPECT_NEAR(links[2].abs_position_[0], 1, 0.001);
  EXPECT_NEAR(links[2].abs_position_[1], 1, 0.001);
  EXPECT_NEAR(links[2].abs_position_[2], 0, 0.001);

  EXPECT_NEAR(links[3].abs_position_[0], 1, 0.001);
  EXPECT_NEAR(links[3].abs_position_[1], 2, 0.001);
  EXPECT_NEAR(links[3].abs_position_[2], 0, 0.001);
}

TEST_F(LinkageTest, ComplexBending)
{
  set_joint(0, 3 * M_PI / 4, 0);
  set_joint(1, M_PI / 2, 0);
  set_joint(2, 0, 0);

  propagate();

  EXPECT_NEAR(links[1].abs_position_[0], 1, 0.001);
  EXPECT_NEAR(links[1].abs_position_[1], 0, 0.001);
  EXPECT_NEAR(links[1].abs_position_[2], 0, 0.001);

  EXPECT_NEAR(links[2].abs_position_[0], 1 - cos(M_PI/4), 0.001);
  EXPECT_NEAR(links[2].abs_position_[1], sin(M_PI/4), 0.001);
  EXPECT_NEAR(links[2].abs_position_[2], 0, 0.001);

  EXPECT_NEAR(links[3].abs_position_[0], 1 - cos(M_PI/4), 0.001);
  EXPECT_NEAR(links[3].abs_position_[1], sin(M_PI/4), 0.001);
  EXPECT_NEAR(links[3].abs_position_[2], -1, 0.001);
}

TEST_F(LinkageTest, AtZeroVelocity)
{
  set_joint(0, 0.4325325, 0);
  set_joint(1, 0.32423423, 0);
  set_joint(2, 0.111, 0);

  propagate();

  EXPECT_NEAR(links[1].abs_velocity_[0], 0, 0.001);
  EXPECT_NEAR(links[1].abs_velocity_[1], 0, 0.001);
  EXPECT_NEAR(links[1].abs_velocity_[2], 0, 0.001);
  EXPECT_NEAR(links[2].abs_velocity_[0], 0, 0.001);
  EXPECT_NEAR(links[2].abs_velocity_[1], 0, 0.001);
  EXPECT_NEAR(links[2].abs_velocity_[2], 0, 0.001);
  EXPECT_NEAR(links[3].abs_velocity_[0], 0, 0.001);
  EXPECT_NEAR(links[3].abs_velocity_[1], 0, 0.001);
  EXPECT_NEAR(links[3].abs_velocity_[2], 0, 0.001);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
