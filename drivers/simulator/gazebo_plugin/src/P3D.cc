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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/Body.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo_plugin/P3D.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("P3D", P3D);


////////////////////////////////////////////////////////////////////////////////
// Constructor
P3D::P3D(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("P3D controller requires a Model as its parent");

  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in P3D \n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
P3D::~P3D()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void P3D::LoadChild(XMLConfigNode *node)
{
   this->myIface = dynamic_cast<PositionIface*>(this->ifaces[0]);

   if (!this->myIface)
      gzthrow("P3D controller requires a Actarray Iface");

  std::string bodyName = node->GetString("bodyName", "", 1);
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));
//  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));

  this->topicName = node->GetString("topicName", "", 1);
  this->frameName = node->GetString("frameName", "", 1);

  std::cout << "==== topic name for P3D ======== " << this->topicName << std::endl;
  rosnode->advertise<std_msgs::Pose3DStamped>(this->topicName);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void P3D::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void P3D::UpdateChild()
{
  Quatern rot;
  Vector3 pos;
  this->myIface->Lock(1);
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime();

  pos = this->myBody->GetPosition();
  rot =  this->myBody->GetRotation();
  
  this->myIface->data->pose.pos.x = pos.x;
  this->myIface->data->pose.pos.y = pos.y;
  this->myIface->data->pose.pos.z = pos.z;

  this->myIface->data->pose.roll  = rot.GetRoll();
  this->myIface->data->pose.pitch = rot.GetPitch();
  this->myIface->data->pose.yaw   = rot.GetYaw();



  this->lock.lock();
  // copy data into pose message
  this->poseMsg.header.frame_id = this->frameName;
  this->poseMsg.header.stamp.sec = (unsigned long)floor(this->myIface->data->head.time);
  this->poseMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->myIface->data->head.time - this->poseMsg.header.stamp.sec) );

  this->poseMsg.pose3D.position.x          = pos.x;
  this->poseMsg.pose3D.position.y          = pos.y;
  this->poseMsg.pose3D.position.z          = pos.z;

  this->poseMsg.pose3D.orientation.x       = rot.x;
  this->poseMsg.pose3D.orientation.y       = rot.y;
  this->poseMsg.pose3D.orientation.z       = rot.z;
  this->poseMsg.pose3D.orientation.w       = rot.u;

  // publish to ros
  rosnode->publish(this->topicName,this->poseMsg);
  this->lock.unlock();




  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void P3D::FiniChild()
{
  // TODO: will be replaced by global ros node eventually
  if (rosnode != NULL)
  {
    std::cout << "shutdown rosnode in P3D" << std::endl;
    //ros::fini();
    rosnode->shutdown();
    //delete rosnode;
  }
}
