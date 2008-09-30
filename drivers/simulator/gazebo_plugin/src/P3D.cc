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
      gzthrow("P3D controller requires a Actarray Iface, though not used.");

  std::string bodyName = node->GetString("bodyName", "", 1);
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));
//  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));

  this->topicName = node->GetString("topicName", "", 1);
  this->frameName = node->GetString("frameName", "", 1);
  this->xyzOffsets  = node->GetVector3("xyzOffsets", Vector3(0,0,0));
  this->rpyOffsets  = node->GetVector3("rpyOffsets", Vector3(0,0,0));

  std::cout << "==== topic name for P3D ======== " << this->topicName << std::endl;
  rosnode->advertise<std_msgs::TransformWithRateStamped>(this->topicName,10);
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

  // apply xyz offsets
  pos = this->myBody->GetPosition() + this->xyzOffsets;

  // apply rpy offsets
  Vector3 rpyTotal;
  rot = this->myBody->GetRotation();
  rpyTotal.x = this->rpyOffsets.x + rot.GetRoll();
  rpyTotal.y = this->rpyOffsets.y + rot.GetPitch();
  rpyTotal.z = this->rpyOffsets.z + rot.GetYaw();
  rot.SetFromEuler(rpyTotal);

  double cur_time = Simulator::Instance()->GetSimTime();
  
  // get Rates
  Vector3 vpos = this->myBody->GetPositionRate(); // get velocity in gazebo frame
  Quatern vrot = this->myBody->GetRotationRate(); // get velocity in gazebo frame
  Vector3 veul = this->myBody->GetEulerRate(); // get velocity in gazebo frame

  this->lock.lock();
  // copy data into pose message
  this->transformMsg.header.frame_id = this->frameName;
  this->transformMsg.header.stamp.sec = (unsigned long)floor(cur_time);
  this->transformMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  cur_time - this->transformMsg.header.stamp.sec) );

  this->transformMsg.transform.translation.x    = pos.x;
  this->transformMsg.transform.translation.y    = pos.y;
  this->transformMsg.transform.translation.z    = pos.z;

  this->transformMsg.transform.rotation.x       = rot.x;
  this->transformMsg.transform.rotation.y       = rot.y;
  this->transformMsg.transform.rotation.z       = rot.z;
  this->transformMsg.transform.rotation.w       = rot.u;

  this->transformMsg.rate.translation.x         = vpos.x;
  this->transformMsg.rate.translation.y         = vpos.y;
  this->transformMsg.rate.translation.z         = vpos.z;

  // pass quaternion
  // this->transformMsg.rate.rotation.x            = vrot.x;
  // this->transformMsg.rate.rotation.y            = vrot.y;
  // this->transformMsg.rate.rotation.z            = vrot.z;
  // this->transformMsg.rate.rotation.w            = vrot.u;

  // pass euler anglular rates
  this->transformMsg.rate.rotation.x            = veul.x;
  this->transformMsg.rate.rotation.y            = veul.y;
  this->transformMsg.rate.rotation.z            = veul.z;
  this->transformMsg.rate.rotation.w            = 0;

  // publish to ros
  rosnode->publish(this->topicName,this->transformMsg);
  this->lock.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void P3D::FiniChild()
{
  rosnode->unadvertise(this->topicName);
}
