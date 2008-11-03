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

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"
#include "phase_space/PhaseSpaceBody.h"

#include "std_msgs/PointStamped.h"

using namespace std ;

namespace pr2_phase_space
{
class PhaseSpacePanTilt : public ros::node
{
public :
  
  PhaseSpacePanTilt() : ros::node("phase_space_pan_tilt")
  {
    subscribe("phase_space_snapshot", snapshot_, &PhaseSpacePanTilt::snapshotCallback, 10) ;
    publish_count_ = 0 ;
  }
  
  ~PhaseSpacePanTilt()
  {
    unsubscribe("phase_space_snapshot") ;
  }
  
  void snapshotCallback()
  {
    if (snapshot_.get_markers_size() > 0)                        // Only execute if we have at least 1 marker in the scene
    {
      if (snapshot_.frameNum % 16 != 0)
        return ;
      
      //! \todo Replace TrackPoint service with a PointStamped message.
      
      /*string topic = "head_controller/track_point" ;
      
      
      pr2_mechanism_controllers::TrackPoint::request req ;
      pr2_mechanism_controllers::TrackPoint::response resp ;
      
      const phase_space::PhaseSpaceMarker& cur_marker = snapshot_.markers[0] ;
      req.target.header = snapshot_.header ;
      req.target.point = cur_marker.location ;
      
      req.target.point.x = cur_marker.location.x ;
      req.target.point.y = -cur_marker.location.z ;
      req.target.point.z = cur_marker.location.y-.8 ;
      req.target.header.frame_id = "torso" ;

      cout << "Requesting to move to: " << req.target.point.x << ","
                                        << req.target.point.y << ","
                                        << req.target.point.z << endl ;

      ros::service::call(topic, req, resp) ;
      cout << "  Resp: Pan=" << resp.pan_angle << "  Tilt:" << resp.tilt_angle << endl ;*/
      
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

  PhaseSpacePanTilt phase_space_pan_tilt ;

  phase_space_pan_tilt.spin() ;

  ros::fini() ;

  printf("**** Exiting ****\n") ;

  return 0 ;
}
