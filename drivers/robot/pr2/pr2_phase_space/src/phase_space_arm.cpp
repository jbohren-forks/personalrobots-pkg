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
 * This node takes the PhaseSpaceSnapshot packet and repackages to work with the pan_tilt tracker
 */

#include <string>

#include "ros/node.h"

// Services
#include "pr2_mechanism_controllers/TrackPoint.h"

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"
#include "phase_space/PhaseSpaceBody.h"

#include "std_msgs/Vector3.h"

using namespace std ;

namespace pr2_phase_space
{
class PhaseSpaceArm : public ros::node
{
public :
  
  PhaseSpaceArm() : ros::node("phase_space_arm")
  {
    subscribe("phase_space_snapshot", snapshot_, &PhaseSpaceArm::snapshotCallback, 10) ;
    advertise<std_msgs::Vector3>("command") ;
    publish_count_ = 0 ;
  }
  
  ~PhaseSpaceArm()
  {
    unsubscribe("phase_space_snapshot") ;
  }
  
  void snapshotCallback()
  {
    if (snapshot_.get_markers_size() > 0)                        // Only execute if we have at least 1 marker in the scene
    {
      if (snapshot_.frameNum % 16 != 0)
        return ;
      
      string topic = "head_controller/track_point" ;
      
      const phase_space::PhaseSpaceMarker& cur_marker = snapshot_.markers[0] ;

      std_msgs::Vector3 cur_cmd ;
      cur_cmd.x = cur_marker.location.x ;
      cur_cmd.y = cur_marker.location.y ;
      cur_cmd.z = cur_marker.location.z ;

      publish("command", cur_cmd) ;
      
      return ;
    }
  }
  
private :
  
  phase_space::PhaseSpaceSnapshot snapshot_ ;
  
  int publish_count_ ;
  
} ;

}


using namespace pr2_phase_space ;


int main(int argc, char** argv)
{
  
  printf("Initializing Ros...") ;
  ros::init(argc, argv);
  printf("Done\n") ;

  PhaseSpaceArm phase_space_arm ;

  phase_space_arm.spin() ;

  ros::fini() ;

  printf("**** Exiting ****\n") ;

  return 0 ;
}
