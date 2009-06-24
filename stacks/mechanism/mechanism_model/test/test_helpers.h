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

#ifndef MECHANISM_MODEL_TEST_HELPERS_H
#define MECHANISM_MODEL_TEST_HELPERS_H

#include <gtest/gtest.h>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include "mechanism_model/robot.h"
#include "mechanism_model/chain.h"
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace mechanism;

bool transformsMatch(const tf::Transform &a, const tf::Transform &b)
{
  return
    a.getOrigin().distance(b.getOrigin()) < 0.00001 &&
    a.getRotation().angle(b.getRotation()) < 0.0001;

}

std::ostream &operator<<(std::ostream &out, const tf::Transform &t)
{
  tf::Vector3 v(t.getOrigin());
  tf::Quaternion q(t.getRotation());
  out << "<" << v[0] << ", " << v[1] << ", " << v[2] << ">";
  out << "<" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ">";
  return out;
}

#define EXPECT_TRANSFORMS_EQ(a, b) EXPECT_PRED2(transformsMatch, a, b)

Joint *quickJoint(const std::string &name, const tf::Vector3 &axis)
{
  Joint *j = new Joint;
  j->name_ = name;
  j->axis_ = axis.normalized();
  j->type_ = JOINT_CONTINUOUS;
  return j;
}

Link *quickLink(const std::string &name, const std::string &parent, const std::string &joint,
                const tf::Vector3 &xyz, const tf::Vector3 &rpy)
{
  Link *l = new Link();
  l->name_ = name;  l->parent_name_ = parent;  l->joint_name_ = joint;
  l->origin_xyz_ = xyz;  l->origin_rpy_ = rpy;
  return l;
}

void setJoint(RobotState *state, int index, double pos, double vel = 0.0)
{
  state->joint_states_[index].position_ = pos;
  state->joint_states_[index].velocity_ = vel;
}

#endif // MECHANISM_MODEL_TEST_HELPERS_H
