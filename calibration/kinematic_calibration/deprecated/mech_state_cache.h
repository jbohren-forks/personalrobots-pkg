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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_MECH_STATE_CACHE_H_
#define KINEMATIC_CALIBRATION_MECH_STATE_CACHE_H_


#include "kinematic_calibration/msg_cache.h"

#include "robot_msgs/MechanismState.h"
#include "kinematic_calibration/Interval.h"

#include "tinyxml/tinyxml.h"

namespace kinematic_calibration
{

class MechStateCache : public MsgCache<robot_msgs::MechanismState>
{
public:
  MechStateCache(unsigned int N) : MsgCache<robot_msgs::MechanismState>(N)
  {

  }

  ~MechStateCache()
  {

  }

  bool initXml(TiXmlElement *config)
  {
    if (!config)
    {
      ROS_ERROR("config==NULL") ;
      return false ;
    }

    TiXmlElement* joint_config ;
    joint_config = config->FirstChildElement("joint") ;
    unsigned int joint_count = 0 ;
    while (joint_config)
    {
      const char* joint_name ;
      double joint_tolerance ;
      int result ;

      joint_name = joint_config->Attribute("name") ;
      result = joint_config->QueryDoubleAttribute("tol", &joint_tolerance) ;
      if (joint_name)
      {
        switch(result)
        {
          case TIXML_SUCCESS:
          {
            ROS_INFO("Joint: %s  Tolerance: %f", joint_name, joint_tolerance) ;

            // Make sure the joint doesn't already exist
            if (joint_map_.find(joint_name) == joint_map_.end())
            {
              JointToleranceElem cur_tol ;
              cur_tol.index = joint_count ;
              cur_tol.pos_tol = joint_tolerance ;
              joint_map_.insert( std::pair<string, JointToleranceElem>(joint_name, cur_tol) ) ;
              joint_count++ ;
            }
            else
            {
              ROS_ERROR("Joint named '%s' already exists!", joint_name) ;
              return false ;
            }
            break ;
          }
          case TIXML_WRONG_TYPE:
            ROS_ERROR("Joint: %s  Tolerance: [WRONG TYPE]", joint_name) ;
            return false ;
          case TIXML_NO_ATTRIBUTE:
            ROS_ERROR("Joint: %s  Tolerance: [NOT FOUND]", joint_name) ;
            return false ;
          default :
            ROS_ERROR("Unknown TinyXML Error Code") ;
            return false ;
        }
      }
      else
      {
        ROS_ERROR("Found joint without a 'name' attribute") ;
        return false ;
      }

      joint_config = joint_config->NextSiblingElement("joint") ;
    }

    return true ;
  }

  bool isStable(const Interval& interval, unsigned int min_samples)
  {
    if (interval.start > interval.end)
      return false ;

    deque<robot_msgs::MechanismState>::iterator it ;
    it = storage_.begin() ;

    // Walk along list to just inside beginning of interval
    while( it < storage_.end() && it->header.stamp < interval.start)
      ++it ;

    std::vector<double> joint_min(joint_map_.size(),  numeric_limits<double>::max()) ;
    std::vector<double> joint_max(joint_map_.size(), -numeric_limits<double>::max()) ;

    // Perform calcs on elems inside interval
    unsigned int sample_count = 0 ;
    std::map<std::string, JointToleranceElem>::iterator joint_map_it ;
    while( it < storage_.end() && it->header.stamp < interval.end)
    {
      for (unsigned int i = 0; i < it->get_joint_states_size(); i++)
      {
        joint_map_it = joint_map_.find(it->joint_states[i].name) ;
        if (joint_map_it != joint_map_.end()) // We found a joint we care about!
        {
          double joint_pos = it->joint_states[i].position ;

          // Update joint min
          if (joint_pos < joint_min[joint_map_it->second.index])
            joint_min[joint_map_it->second.index] = joint_pos ;

          // Update joint max
          if (joint_pos > joint_max[joint_map_it->second.index])
            joint_max[joint_map_it->second.index] = joint_pos ;
        }
      }
      sample_count++ ;
    }

    printf("Sample Count: %u\n", sample_count) ;
    if (sample_count < min_samples)
      return false ;

    printf("Joints:\n") ;
    for (unsigned int i=0; i<joint_min.size(); i++)
    {
      printf("%u) %f -> %f\n", i, joint_min[i], joint_max[i]) ;
    }

    while(joint_map_it != joint_map_.end())
    {
      double range = joint_max[joint_map_it->second.index] - joint_min[joint_map_it->second.index] ;

      // Check if we're past the allowed tolerance for this joint
      if (range > joint_map_it->second.pos_tol)
        return false ;

      ++joint_map_it ;
    }

    return true ;
  }


private :
  struct JointToleranceElem
  {
    unsigned int index ;
    double pos_tol ;
  } ;

  std::map<std::string, JointToleranceElem> joint_map_ ;

} ;

}


#endif // KINEMATIC_CALIBRATION_MECH_STATE_CACHE_H_
