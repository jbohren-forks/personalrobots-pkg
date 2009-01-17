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


#ifndef KDL_CHAIN_EFFSOLVER_HPP
#define KDL_CHAIN_EFFSOLVER_HPP

#include "chain.hpp"
#include "jntarray.hpp"
#include "chainjnttojacsolver.hpp"
#include "chainfksolverpos_recursive.hpp"


namespace KDL {

    /**
	  * \brief This <strong>abstract</strong> class encapsulates a
	  * solver to convert joint efforts into a wrench
     *
     * @ingroup KinematicFamily
     */

    //Forward definition
    class ChainFEffSolver {
    public:
        /**
         * Calculate forward position kinematics for a KDL::Chain,
         * from joint coordinates to cartesian pose.
         *
         * @param eff input joint efforts
         * @param wrench_out output wrench
         *
         * @return if < 0 something went wrong
         */
        int JntToCart(const JntArray& q, const JntArray& eff, Wrench& wrench_out);
        ChainFEffSolver(const Chain& chain);
        ~ChainFEffSolver(){};

    private:
      unsigned int num_joints_, num_segments_;
      Chain chain_;
    };




    /**
	  * \brief This <strong>abstract</strong> class encapsulates a
	  * solver to convert a wrench into joint efforts
     *
     * @ingroup KinematicFamily
     */

    //inverse definition
    class ChainIEffSolver {
    public:
        /**
         * Calculate forward position kinematics for a KDL::Chain,
         * from joint coordinates to cartesian pose.
         *
         * @param wrench_in vector of wrenches applied to each segment. Ref frame base, ref point segment
         * @param eff_out the output joint efforts
         *
         * @return if < 0 something went wrong
         */
        int CartToJnt(const JntArray& q, const std::vector<Wrench>& wrench_in, JntArray& eff_out);
        int CartToJnt(const JntArray& q, const Wrench wrench_in, JntArray& eff_out, int segment_nr=-1);
        ChainIEffSolver(const Chain& chain);
        ~ChainIEffSolver(){};

    private:
      unsigned int num_joints_, num_segments_;
      Jacobian jacobian_;
      Chain chain_;
      std::vector<Wrench> wrenches_;
    };


}//end of namespace KDL

#endif
