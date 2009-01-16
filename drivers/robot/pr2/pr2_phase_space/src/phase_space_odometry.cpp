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
 * This node takes the PhaseSpaceSnapshot packet and repackages into a form that can be used with the odometry
 */

#include "ros/node.h"

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"
#include "phase_space/PhaseSpaceBody.h"

#include "std_msgs/TransformWithRateStamped.h"

namespace pr2_phase_space
{
class PhaseSpaceOdometry : public ros::Node
{
public :
  
  PhaseSpaceOdometry() : ros::Node("phase_space_odometry")
  {
    advertise<std_msgs::TransformWithRateStamped>("phase_space_odom", 10) ;
    subscribe("phase_space_snapshot", snapshot_, &PhaseSpaceOdometry::snapshotCallback, 10) ;
    publish_count_ = 0 ;
  }
  
  ~PhaseSpaceOdometry()
  {
    unsubscribe("phase_space_snapshot") ;
    unadvertise("phase_space_odom") ;
  }
  
  void snapshotCallback()
  {
    if (snapshot_.get_bodies_size() > 0)                        // Only execute if we have at least 1 body in the scene
    {
      std_msgs::TransformWithRateStamped odom ;
      odom.header = snapshot_.header ;
      odom.transform = snapshot_.bodies[0].pose ;               // We're not matching IDs or anything fancy. We're simply grabing the first body we see.
      publish("phase_space_odom", odom) ;
      
      if (publish_count_% 480 == 0)
        printf("Published %u messages\n", publish_count_) ;
      
      publish_count_++ ;  
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

  PhaseSpaceOdometry phase_space_odometry ;

  phase_space_odometry.spin() ;

  ros::fini() ;

  printf("**** Exiting ****\n") ;

  return 0 ;
}
