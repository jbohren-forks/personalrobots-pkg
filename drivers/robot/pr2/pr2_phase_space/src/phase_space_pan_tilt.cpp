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

/****
 * This node takes the MocapSnapshot packet and repackages to work with the pan_tilt tracker
 */

#include <string>

#include "ros/node.h"

// Messages
#include "robot_msgs/MocapSnapshot.h"
#include "robot_msgs/MocapMarker.h"
#include "robot_msgs/MocapBody.h"

#include "std_msgs/PointStamped.h"

using namespace std ;

namespace pr2_phase_space
{
class PhaseSpacePanTilt : public ros::node
{
public :
  
  PhaseSpacePanTilt() : ros::node("phase_space_pan_tilt"), topic("head_controller/track_point")
  {
    subscribe("phase_space_snapshot", snapshot_, &PhaseSpacePanTilt::snapshotCallback, 10) ;
    advertise<std_msgs::PointStamped>(topic, 10) ;
  }
  
  ~PhaseSpacePanTilt()
  {
    unsubscribe("phase_space_snapshot") ;
    unadvertise(topic) ;
  }
  
  void snapshotCallback()
  {
    if (snapshot_.get_markers_size() > 0)                        // Only execute if we have at least 1 marker in the scene
    {
      if (snapshot_.frameNum % 16 != 0)
        return ;
      
      if (snapshot_.get_markers_size() == 0)
        return ;

      robot_msgs::MocapMarker& cur_marker = snapshot_.markers[0] ;
      
      std_msgs::PointStamped target_point ;
      target_point.header = snapshot_.header ;
      target_point.point = cur_marker.location ;
      
      publish(topic, target_point) ;

      return ;
    }
  }
  
private :
  const string topic ;
  robot_msgs::MocapSnapshot snapshot_ ;
  
  int publish_count_ ;
  
} ;

}


using namespace pr2_phase_space ;


int main(int argc, char** argv)
{
  
  printf("Initializing Ros...") ;
  ros::init(argc, argv);
  printf("Done\n") ;

  PhaseSpacePanTilt phase_space_pan_tilt ;

  phase_space_pan_tilt.spin() ;

  ros::fini() ;

  printf("**** Exiting ****\n") ;

  return 0 ;
}
