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

#include <kdl/chainfksolvervelfull_recursive.hpp>

namespace KDL
{

ChainFkSolverVelFull_recursive::ChainFkSolverVelFull_recursive(const Chain& _chain):
  chain(_chain)
{
}

ChainFkSolverVelFull_recursive::~ChainFkSolverVelFull_recursive()
{
}

int ChainFkSolverVelFull_recursive::JntToCart(const JntArrayVel& in, std::vector<FrameVel>& out_vec, int segmentNr)
{

    if(segmentNr<0)
         segmentNr=chain.getNrOfSegments();

    FrameVel out=FrameVel::Identity();

    if(!(in.q.rows()==chain.getNrOfJoints()&&in.qdot.rows()==chain.getNrOfJoints()))
        return -1;
    else if(segmentNr>int(chain.getNrOfSegments()))
        return -1;
    else if (int(out_vec.size()) < segmentNr)
        return -1;
    else{
        int j=0;
        for (int i=0;i<segmentNr;i++) {
            //Calculate new Frame_base_ee
            if(chain.getSegment(i).getJoint().getType()!=Joint::None){
                out=out*FrameVel(chain.getSegment(i).pose(in.q(j)),
                                 chain.getSegment(i).twist(in.q(j),in.qdot(j)));
                j++;//Only increase jointnr if the segment has a joint
            }else{
                out=out*FrameVel(chain.getSegment(i).pose(0.0),
                                 chain.getSegment(i).twist(0.0,0.0));
            }
            out_vec[i] = out;
        }
        return 0;
    }
}

} // namespace KDL
