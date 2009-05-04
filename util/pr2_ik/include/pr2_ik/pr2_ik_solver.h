//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <mechanism_model/robot.h>
#include <mechanism_model/chain.h>

#include <urdf/parser.h>
#include <pr2_ik/pr2_ik.h>

#include <kdl/chainiksolver.hpp>
#include <Eigen/Array>

namespace pr2_ik
{
  class PR2IKSolver : public KDL::ChainIkSolverPos
  {
    public:
    
    PR2IKSolver();

    ~PR2IKSolver();

    PR2IK *pr2_ik_;

    bool active_;

    mechanism::Chain chain_;

    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);

    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out);
  
    int CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out, const double &timeout);

    private:

    mechanism::Robot robot_model_;

    double distance(const tf::Transform &transform);

    Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);

    double computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2);

    bool getCount(int &count, int max_count, int min_count);

    double search_discretization_angle_;
  };
}
