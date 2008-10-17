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

//! \author Sachin Chitta

/****
 * This node takes the PhaseSpaceSnapshot packet and repackages into a form that can be used with the odometry
 */

#include "ros/node.h"

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"
#include "phase_space/PhaseSpaceBody.h"

#include "std_msgs/Transform.h"
#include "std_msgs/TransformWithRateStamped.h"
#include "std_msgs/RobotBase2DOdom.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


namespace pr2_phase_space
{
  class PhaseSpaceLocalization : public ros::node
  {
    public :
  
      PhaseSpaceLocalization() : ros::node("phase_space_localization")
      {
	advertise<std_msgs::RobotBase2DOdom>("localizedpose");

        m_tfServer = new tf::TransformBroadcaster(*this);

        subscribe("phase_space_snapshot", snapshot_, &PhaseSpaceLocalization::snapshotCallback, 10) ;
        publish_count_ = 0 ;
      }
  
      ~PhaseSpaceLocalization()
      {
        unsubscribe("phase_space_snapshot") ;
        unadvertise("localizedpose") ;
	if (m_tfServer)
	    delete m_tfServer; 
      }
  
      void snapshotCallback()
      {
        if (snapshot_.get_bodies_size() > 0)                        // Only execute if we have at least 1 body in the scene
        {
          m_currentPos.header = snapshot_.header;
          m_currentPos.header.frame_id = "map";

          m_currentPos.pos.x = snapshot_.bodies[0].pose.translation.x;
          m_currentPos.pos.y = snapshot_.bodies[0].pose.translation.y;

          double yaw,pitch,roll;

          tf::Transform mytf;

          tf::TransformMsgToTF(snapshot_.bodies[0].pose,mytf);

          mytf.getBasis().getEulerZYX(yaw,pitch,roll);
 
          m_currentPos.pos.th = yaw;

          publish("localizedpose", m_currentPos) ;

          m_tfServer->sendTransform(mytf,m_currentPos.header.stamp,"base","map");
      
          if (publish_count_% 480 == 0)
            printf("Published %u messages\n", publish_count_) ;
      
          publish_count_++ ;  
        }
      }
  
    private:
  
      phase_space::PhaseSpaceSnapshot snapshot_;
      std_msgs::RobotBase2DOdom m_currentPos;
      tf::TransformBroadcaster *m_tfServer;
      int publish_count_ ;
  
  } ;

}

using namespace pr2_phase_space ;

int main(int argc, char** argv)
{
  
  printf("Initializing Ros...") ;
  ros::init(argc, argv);
  printf("Done\n") ;

  PhaseSpaceLocalization phase_space_loc;

  phase_space_loc.spin() ;

  ros::fini() ;

  printf("**** Exiting ****\n") ;

  return 0 ;
}
