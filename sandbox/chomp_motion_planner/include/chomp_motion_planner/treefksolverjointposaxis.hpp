// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Wim Meeussen <meeussen at willowgarage dot com>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDLTREEFKSOLVERJOINTPOSAXIS_HPP
#define KDLTREEFKSOLVERJOINTPOSAXIS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

namespace KDL {

class TreeFkSolverJointPosAxis

{
public:
  TreeFkSolverJointPosAxis(const Tree& tree);
  ~TreeFkSolverJointPosAxis();

  int JntToCart(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames) const;

  const std::vector<std::string> getSegmentNames() const;
  const std::map<std::string, int> getSegmentNameToIndex() const;

private:
  int treeRecursiveFK(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames,
      const Frame& previous_frame, const SegmentMap::const_iterator this_segment, int segment_nr) const;

  std::vector<std::string> segment_names_;
  std::map<std::string, int> segment_name_to_index_;
  Tree tree_;

  void assignSegmentNumber(const SegmentMap::const_iterator this_segment);

};

} // namespace KDL

#endif
