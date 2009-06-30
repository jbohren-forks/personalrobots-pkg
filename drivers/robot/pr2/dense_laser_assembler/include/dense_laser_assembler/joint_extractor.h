/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef DENSE_LASER_ASSEMBLER_JOINT_EXTRACTOR
#define DENSE_LASER_ASSEMBLER_JOINT_EXTRACTOR

#include <vector>

#include "mechanism_msgs/MechanismState.h"

namespace dense_laser_assembler
{

/**
 * Streams an array of joint positions from MechanismState, given a list of requested joints
 */
class JointExtractor
{
public:

  struct JointArray
  {
    roslib::Header header ;
    std::vector<double> positions ;
  } ;

  JointExtractor()
  {
    joint_names_.clear() ;
  }

  JointExtractor(std::vector<std::string> joint_names)
  {
    joint_names_ = joint_names ;
  }

  void setJointNames(std::vector<std::string> joint_names)
  {
    joint_names_ = joint_names ;
  }

  /**
   * Links Consumer's input to some provider's output.
   * \param provider The filter from which we want to receive data
   */
  template<class T>
  void subscribe(T& provider)
  {
    provider.addOutputCallback(boost::bind(&JointExtractor::processMechState, this, _1)) ;
  }

  void processMechState(const mechanism_msgs::MechanismStateConstPtr& msg)
  {
    //printf("Processing MechState\n") ;
    boost::shared_ptr<JointArray> joint_array_ptr(new JointArray) ;
    joint_array_ptr->positions.resize(joint_names_.size()) ;

    joint_array_ptr->header = msg->header ;

    //! \todo This can seriously be optimized using a map, or some sort of cached lookup
    for (unsigned int i=0; i<joint_names_.size(); i++)
    {
      for (unsigned int j=0; j<msg->joint_states.size(); j++)
      {
        if (joint_names_[i] == msg->joint_states[j].name )
        {
          joint_array_ptr->positions[i] = msg->joint_states[j].position ;
          break ;
        }
      }
    }

    // Sequentially call each registered call
    for (unsigned int i=0; i<output_callbacks_.size(); i++)
      output_callbacks_[i](joint_array_ptr) ;
  }

  void addOutputCallback(const boost::function<void(const boost::shared_ptr<JointArray const>&)>& callback)
  {
    output_callbacks_.push_back(callback) ;
  }

private:
  std::vector<std::string> joint_names_ ;
  std::vector<boost::function<void(const boost::shared_ptr<JointArray const>&)> > output_callbacks_ ;

} ;

}








#endif // DENSE_LASER_ASSEMBLER_JOINT_EXTRACTOR
