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

/**

@mainpage

@htmlinclude manifest.html

@b odom_localization is simply forwards the odometry information 

<hr>

@section usage Usage
@verbatim
$ odom_localization
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "odom"/RobotBase2DOdom : robot's odometric pose.  Only the position information is used (velocity is ignored).

Publishes to (name / type):
- @b "localizedpose"/RobotBase2DOdom : robot's localized map pose.  Only the position information is set (no velocity).
- @b "particlecloud"/ParticleCloud2D : fake set of particles being maintained by the filter (one paricle only).

<hr>

@section parameters ROS parameters

- None

 **/

#include <ros/node.h>

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/ParticleCloud2D.h>
#include <std_msgs/Pose2DFloat32.h>

#include <rosTF/rosTF.h>


class OdomNode: public ros::node
{
public:
    OdomNode(void) : ros::node("odom_localization")
    {
	advertise<std_msgs::RobotBase2DOdom>("localizedpose");
	advertise<std_msgs::ParticleCloud2D>("particlecloud");
	
	subscribe("odom", m_odomMsg, &AmclNode::odomReceived);
    }
    
    ~OdomNode(void)
    {
    }
    
    
private:
    
    rosTFServer              *m_tf;
    
    std_msgs::RobotBase2DOdom m_odomMsg;
    
    void odomReceived(void);
};

int main(int argc, char** argv)
{
    ros::init(argc, argv);
    
    OdomNode odom;
    odom.spin();
    odom.shutdown();
    
    return 0;
}
