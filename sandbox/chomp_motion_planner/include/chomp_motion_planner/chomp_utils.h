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

namespace chomp
{

/**
 * \brief Takes in an std::vector of joint value messages, and writes them out into the KDL joint array.
 *
 * The template typename T needs to be an std::vector of some message which has an std::string "joint_name"
 * and a double array "value".
 *
 * Names to KDL joint index mappings are performed using the given ChompRobotModel.
 */
template<typename T>
void jointMsgToArray(T& msg_vector, KDL::JntArray& joint_array, ChompRobotModel& robot_model)
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

} //namespace chomp

#endif /* CHOMP_UTILS_H_ */