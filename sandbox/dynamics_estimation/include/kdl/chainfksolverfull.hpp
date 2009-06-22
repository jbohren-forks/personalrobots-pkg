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

#ifndef KDL_CHAIN_FKSOLVERFULL_HPP
#define KDL_CHAIN_FKSOLVERFULL_HPP

#include "kdl/chain.hpp"
#include "kdl/framevel.hpp"
#include "kdl/frameacc.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jntarrayvel.hpp"
#include "kdl/jntarrayacc.hpp"
#include <vector>

namespace KDL {

    /**
     * \brief This <strong>abstract</strong> class encapsulates a
     * solver for the forward position kinematics for a KDL::Chain,
     * for all segments along the chain.
     *
     * @ingroup KinematicFamily
     */

    //Forward definition
    class ChainFkSolverPosFull {
    public:
        /**
         * Calculate forward position kinematics for a KDL::Chain,
         * from joint coordinates to cartesian pose
         *
         * @param q_in input joint coordinates
         * @param p_out reference to vector of output cartesian poses
         * @param segmentNr Last segment number to compute (defaults to -1,
         *      which means compute for all segments)
         *
         * @return if < 0 something went wrong
         */
        virtual int JntToCart(const JntArray& q_in, std::vector<Frame>& p_out, int segmentNr=-1)=0;
        virtual ~ChainFkSolverPosFull(){};
    };

    /**
     * \brief This <strong>abstract</strong> class encapsulates a solver
     * for the forward velocity kinematics for a KDL::Chain, for all segments
     * along the chain.
     *
     * @ingroup KinematicFamily
     */
    class ChainFkSolverVelFull {
    public:
        /**
         * Calculate forward position and velocity kinematics, from
         * joint coordinates to cartesian coordinates.
         *
         * @param q_in input joint coordinates (position and velocity)
         * @param out reference to vector of output cartesian coordinates (position and velocity)
         * @param segmentNr Last segment number to compute (defaults to -1,
         *      which means compute for all segments)
         *
         * @return if < 0 something went wrong
         */
        virtual int JntToCart(const JntArrayVel& q_in, std::vector<FrameVel>& out, int segmentNr=-1)=0;

        virtual ~ChainFkSolverVelFull(){};
    };

    /**
     * \brief This <strong>abstract</strong> class encapsulates a solver
     * for the forward acceleration kinematics for a KDL::Chain, for all
     * segments along the chain.
     *
     * @ingroup KinematicFamily
     */

    class ChainFkSolverAccFull {
    public:
        /**
         * Calculate forward position, velocity and accelaration
         * kinematics, from joint coordinates to cartesian coordinates
         *
         * @param q_in input joint coordinates (position, velocity and
         * acceleration
         * @param out reference to vector of output cartesian coordinates (position, velocity
         * and acceleration
         * @param segmentNr Last segment number to compute (defaults to -1,
         *      which means compute for all segments)
         *
         * @return if < 0 something went wrong
         */
        virtual int JntToCart(const JntArrayAcc& q_in, std::vector<FrameAcc>& out, int segmentNr=-1)=0;
        virtual ~ChainFkSolverAccFull()=0;
    };


}//end of namespace KDL

#endif
