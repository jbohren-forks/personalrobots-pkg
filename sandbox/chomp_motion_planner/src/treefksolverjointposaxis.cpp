/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Wim Meeussen, Mrinal Kalakrishnan */

#include <chomp_motion_planner/treefksolverjointposaxis.hpp>
#include <iostream>

using namespace std;

namespace KDL {

TreeFkSolverJointPosAxis::TreeFkSolverJointPosAxis(const Tree& tree):
  tree_(tree)
{
  segment_names_.clear();
  assignSegmentNumber(tree_.getSegment("root"));
}

TreeFkSolverJointPosAxis::~TreeFkSolverJointPosAxis()
{
}

int TreeFkSolverJointPosAxis::JntToCart(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames) const
{
  joint_pos.resize(tree_.getNrOfJoints());
  joint_axis.resize(tree_.getNrOfJoints());
  segment_frames.resize(segment_names_.size());

  // start the recursion
  treeRecursiveFK(q_in, joint_pos, joint_axis, segment_frames, Frame::Identity(), tree_.getSegment("root"), 0);

  return 0;
}

int TreeFkSolverJointPosAxis::treeRecursiveFK(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames,
    const Frame& previous_frame, const SegmentMap::const_iterator this_segment, int segment_nr) const
{
  Frame this_frame = previous_frame;

  // get the joint angle:
  double jnt_p = 0;
  if (this_segment->second.segment.getJoint().getType() != Joint::None)
  {
    int q_nr = this_segment->second.q_nr;
    jnt_p = q_in(q_nr);
    joint_pos[q_nr] = this_frame * this_segment->second.segment.getJoint().JointOrigin();
    joint_axis[q_nr] = this_frame.M * this_segment->second.segment.getJoint().JointAxis();
  }

  // do the FK:
  this_frame = this_frame * this_segment->second.segment.pose(jnt_p);
  segment_frames[segment_nr] = this_frame;
  segment_nr++;

  // get poses of child segments
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++)
    segment_nr = treeRecursiveFK(q_in, joint_pos, joint_axis, segment_frames, this_frame, *child, segment_nr);
  return segment_nr;
}

void TreeFkSolverJointPosAxis::assignSegmentNumber(const SegmentMap::const_iterator this_segment)
{
  int num = segment_names_.size();
  segment_names_.push_back(this_segment->first);
  segment_name_to_index_[this_segment->first] = num;

  // add the child segments recursively
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++)
  {
    assignSegmentNumber(*child);
  }
}

const std::vector<std::string> TreeFkSolverJointPosAxis::getSegmentNames() const
{
  return segment_names_;
}

const std::map<std::string, int> TreeFkSolverJointPosAxis::getSegmentNameToIndex() const
{
  return segment_name_to_index_;
}

} // namespace KDL
