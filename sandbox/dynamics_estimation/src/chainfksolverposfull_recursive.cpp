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

#include <kdl/chainfksolverposfull_recursive.hpp>

namespace KDL
{

ChainFkSolverPosFull_recursive::ChainFkSolverPosFull_recursive(const Chain& _chain):
  chain(_chain)
{
}

ChainFkSolverPosFull_recursive::~ChainFkSolverPosFull_recursive()
{
}

int ChainFkSolverPosFull_recursive::JntToCart(const JntArray& q_in, std::vector<Frame>& p_out_vec, int segmentNr)
{
    if(segmentNr<0)
         segmentNr=chain.getNrOfSegments();

    Frame p_out = Frame::Identity();

    if(q_in.rows()!=chain.getNrOfJoints())
        return -1;
    else if(segmentNr>int(chain.getNrOfSegments()))
        return -1;
    else if (int(p_out_vec.size()) < segmentNr)
        return -1;
    else{
        int j=0;
        for(int i=0;i<segmentNr;i++){
            if(chain.getSegment(i).getJoint().getType()!=Joint::None){
                p_out = p_out*chain.getSegment(i).pose(q_in(j));
                j++;
            }else{
                p_out = p_out*chain.getSegment(i).pose(0.0);
            }
            p_out_vec[i] = p_out;
        }
        return 0;
    }
}

} // namespace KDL
