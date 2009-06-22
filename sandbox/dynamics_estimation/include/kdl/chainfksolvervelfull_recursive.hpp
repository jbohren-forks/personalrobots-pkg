// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Mrinal Kalakrishnan <kalakris at willowgarage dot com>
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


#ifndef KDLCHAINFKSOLVERVELFULL_RECURSIVE_HPP_
#define KDLCHAINFKSOLVERVELFULL_RECURSIVE_HPP_

#include <kdl/chainfksolverfull.hpp>

namespace KDL
{

/**
 * Implementation of a recursive forward velocity kinematics
 * algorithm to calculate the velocity transformation from joint
 * space to Cartesian space of a general kinematic chain (KDL::Chain).
 *
 * @ingroup KinematicFamily
 */
class ChainFkSolverVelFull_recursive: public ChainFkSolverVelFull
{
public:
  ChainFkSolverVelFull_recursive(const Chain& chain);
  virtual int JntToCart(const JntArrayVel& q_in, std::vector<FrameVel>& out, int segmentNr=-1);
  virtual ~ChainFkSolverVelFull_recursive();

private:
  const Chain chain;
};

}

#endif /* KDLCHAINFKSOLVERVELFULL_RECURSIVE_HPP_ */
