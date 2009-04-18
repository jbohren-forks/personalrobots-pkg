/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Controller for diffdrive robots in gazebo.
 * Author: Tony Pratkanis
 * Date: 17 April 2009
 * SVN info: $Id$
 */

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <ros/node.h>
#include <robot_msgs/PoseDot.h>

gazebo::PositionIface *posIface = NULL;
robot_msgs::PoseDot cmd_msg;

class DiffDrive {
public:
  gazebo::PositionIface *posIface;
  robot_msgs::PoseDot cmd_msg;
  
  void cmdVelCallBack() {
    if (posIface) {
      posIface->Lock(1);
      posIface->data->cmdVelocity.pos.x = cmd_msg.vel.vx;
      posIface->data->cmdVelocity.pos.y = cmd_msg.vel.vy;
      posIface->data->cmdVelocity.yaw = cmd_msg.ang_vel.vz;
      posIface->Unlock();
    }
  }

  DiffDrive() {
    gazebo::Client *client = new gazebo::Client();
    gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
    posIface = new gazebo::PositionIface();
  
    int serverId = 0;
    
    /// Connect to the libgazebo server
    try {
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
      return;
    }
    
    /// Open the Simulation Interface
    try {
      simIface->Open(client, "default");
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
      return;
  }
    
    /// Open the Position interface
    try {
      posIface->Open(client, "position_iface_0");
    } catch (std::string e) {
      std::cout << "Gazebo error: Unable to connect to the position interface\n" << e << "\n";
      return;
    }
    
    // Enable the motor
    posIface->Lock(1);
    posIface->data->cmdEnableMotors = 1;
    posIface->Unlock();
    
    ros::Node n("gazebo_diffdrive");
    ros::Node::instance()->subscribe("/cmd_vel", cmd_msg, &DiffDrive::cmdVelCallBack, this, 10);
    
    n.spin();
  }
};




int main(int argc, char** argv) {
  ros::init(argc, argv);
  DiffDrive d;
  return 0;
}


