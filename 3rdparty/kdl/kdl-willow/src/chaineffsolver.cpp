// Copyright  (C)  2009 Willow Garage <meeussen at willowgarage dot com>

// Version: 1.0
// Author: Wim Meeussen
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



#include "chaineffsolver.hpp"
#include <iostream>

using namespace std;

namespace KDL {


  ChainFEffSolver::ChainFEffSolver(const Chain& chain)
    : num_joints_(chain.getNrOfJoints()),
      num_segments_(chain.getNrOfSegments()),
      chain_(chain)
  {}


  int ChainFEffSolver::JntToCart(const JntArray& q, const JntArray& eff, Wrench& wrench_out)
  {
    assert(q.rows() == num_joints_);
    assert(eff.rows() == num_joints_);
    wrench_out = Wrench::Zero();

    Frame F_seg_seg;
    Frame F_seg_ee = Frame::Identity();
    Vector v_seg_ee = Vector::Zero();
    Wrench w_sum = Wrench::Zero();

    int j=0;

    for (unsigned int i=num_segments_-1;i<=0;i--) {
      // frame from this segment to next segment
      F_seg_seg = chain_.getSegment(i).pose(q(j));
      
      // vector from this segment to ee
      v_seg_ee = F_seg_ee.M  * v_seg_ee;

      // convert w_sum to reference frame in current segment
      w_sum = F_seg_seg.M * w_sum;
      
      // add wrench from current joint with refpoint ee
      if (chain_.getSegment(i).getJoint().getType()!=Joint::None){
	w_sum += chain_.getSegment(i).getJoint().wrench(eff(j)).RefPoint(v_seg_ee);
	j++;
      }
    }      

    return 0;
  }







  ChainIEffSolver::ChainIEffSolver(const Chain& chain)
    : num_joints_(chain.getNrOfJoints()),
      num_segments_(chain.getNrOfSegments()),
      jacobian_(num_joints_, num_segments_),
      chain_(chain),
      wrenches_(num_segments_)

  {
    // set all wrenches to 0
    for (unsigned int i=0; i<num_segments_; i++)
      wrenches_[i] = Wrench::Zero();
}


  int ChainIEffSolver::CartToJnt(const JntArray& q, const Wrench wrench_in, JntArray& eff_out, int segment_nr)
  {
    assert(segment_nr < num_segments_);
    assert(segment_nr >= -1);
    if (segment_nr == -1) segment_nr = num_segments_ -1;

    wrenches_[segment_nr] = wrench_in;
    int res = CartToJnt(q, wrenches_, eff_out);
    wrenches_[segment_nr] = Wrench::Zero();

    return res;
  }


  int ChainIEffSolver::CartToJnt(const JntArray& q, const vector<Wrench>& wrench_in, JntArray& eff_out)
  {
    assert(q.rows() == num_joints_);
    assert(wrench_in.size() == num_segments_);

    Jacobian jac(num_joints_, num_segments_);
    Frame T_tmp = Frame::Identity();
    Twist t_tmp = Twist::Zero();

    for (unsigned int j=0; j<num_joints_; j++)
      eff_out(j) = 0;

    int j=0;
    Frame total;

    for (unsigned int i=0;i<num_segments_;i++) {
      // segment has a joint
      if (chain_.getSegment(i).getJoint().getType()!=Joint::None){
	total = T_tmp * chain_.getSegment(i).pose(q(j));

	//changing base of new segment's twist to base frame
	t_tmp = T_tmp.M * chain_.getSegment(i).twist(q(j), 1.0);
      }
      // segment has no joint
      else
	total = T_tmp * chain_.getSegment(i).pose(0.0);

      // Changing Refpoint of all jac columns to new ee 
      changeRefPoint(jac, total.p - T_tmp.p, jac);

      // Only increase jointnr if the segment has a joint
      if(chain_.getSegment(i).getJoint().getType()!=Joint::None){
	jac.twists[j] = t_tmp;
	j++;
      }
      T_tmp = total;

      // calculate torque caused by wrench i in joints 0->j
      for (unsigned int k=0; k<j; k++)
	for (unsigned int l=0; l<6; l++)
	  eff_out(k) += (jac(l,k) * wrench_in[i](l)); 
    }
    return 0;
  }


}//end of namespace KDL
