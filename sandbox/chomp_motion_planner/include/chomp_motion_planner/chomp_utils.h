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

/** \author Mrinal Kalakrishnan */


#ifndef CHOMP_UTILS_H_
#define CHOMP_UTILS_H_

#include <kdl/jntarray.hpp>
#include <chomp_motion_planner/chomp_robot_model.h>
#include <iostream>
#include <Eigen/Core>

namespace chomp
{

static const int DIFF_RULE_LENGTH = 7;

// the differentiation rules (centered at the center)
static const double DIFF_RULES[3][DIFF_RULE_LENGTH] = {
    {0, 0, -2/6.0, -3/6.0, 6/6.0, -1/6.0, 0},
    {0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0},
    {0, 1/12.0, -17/12.0, 46/12.0, -46/12.0, 17/12.0, -1/12.0}
};

/**
 * \brief Takes in an std::vector of joint value messages, and writes them out into the KDL joint array.
 *
 * The template typename T needs to be an std::vector of some message which has an std::string "joint_name"
 * and a double array/vector "value".
 *
 * Names to KDL joint index mappings are performed using the given ChompRobotModel.
 */
template<typename T>
void jointMsgToArray(T& msg_vector, Eigen::MatrixXd::RowXpr joint_array, ChompRobotModel& robot_model)
{
  for (typename T::iterator it=msg_vector.begin(); it!=msg_vector.end(); it++)
  {
    std::string name = it->joint_name;
    int kdl_number = robot_model.urdfNameToKdlNumber(name);
    if (kdl_number>=0)
      joint_array(kdl_number) = it->value[0];   //@TODO we assume a single joint value per joint now
  }
}

inline void debugJointArray(KDL::JntArray& joint_array)
{
  for (unsigned int i=0; i<joint_array.rows(); i++)
  {
    std::cout << joint_array(i) << "\t";
  }
  std::cout << std::endl;
}

/*template<typename KDLType, typename EigenType>
void kdlToEigen(KDLType& kdl_t, Eigen::Map<EigenType>& eigen_t, int rows, int cols)
{
  eigen_t = Eigen::Map<EigenType>(kdl_t.data, rows, cols);
}*/


template<typename KDLType, typename EigenType>
void kdlVecToEigenVec(std::vector<KDLType>& kdl_v, std::vector<Eigen::Map<EigenType> >& eigen_v, int rows, int cols)
{
  int size = kdl_v.size();
  //typename Eigen::Map<EigenType>::Scalar dummy[rows*cols];
  //eigen_v.resize(size, Eigen::Map<EigenType>(dummy, rows, cols));
  eigen_v.clear();
  for (int i=0; i<size; i++)
  {
    //kdlToEigen(kdl_v[i], eigen_v[i], rows, cols);
    eigen_v.push_back(Eigen::Map<EigenType>(kdl_v[i].data, rows, cols));
  }
}


template<typename KDLType, typename EigenType>
void kdlVecVecToEigenVecVec(std::vector<std::vector<KDLType> >& kdl_vv, std::vector<std::vector<Eigen::Map<EigenType> > > & eigen_vv, int rows, int cols)
{
  int size = kdl_vv.size();
  eigen_vv.resize(size);
  for (int i=0; i<size; i++)
  {
    kdlVecToEigenVec(kdl_vv[i], eigen_vv[i], rows, cols);
  }
}


} //namespace chomp

#endif /* CHOMP_UTILS_H_ */
