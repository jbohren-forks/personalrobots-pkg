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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */


#include <algorithm>
#include <assert.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo_plugin/Ros_Node.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_node", Ros_Node);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Ros_Node::Ros_Node(Entity *parent)
    : Controller(parent)
{
  this->myBodyParent = dynamic_cast<Body*>(this->parent);

  if (!this->myBodyParent)
    gzthrow("Ros_Node controller requires a Body as its parent");

  //  int argc = 0;
  //  char** argv = NULL;
  //  // start a node
  //  ros::init(argc,argv);
  //  this->nodeName = node->GetString("nodeName","default_gazebo_ros_node",0); //read from xml file
  //  rosnode = new ros::node(this->nodeName);

  rosnode = ros::g_node; // comes from where?
  int argc;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::node("ros_gazebo");
  }



}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Ros_Node::~Ros_Node()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Ros_Node::LoadChild(XMLConfigNode *node)
{
  //this->cameraIface = dynamic_cast<CameraIface*>(this->ifaces[0]);

  //if (!this->cameraIface)
  //  gzthrow("Ros_Node controller requires a CameraIface");


}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Ros_Node::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Ros_Node::UpdateChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Ros_Node::FiniChild()
{
  // TODO: will be replaced by global ros node eventually
  delete rosnode;
  ros::fini();
}



