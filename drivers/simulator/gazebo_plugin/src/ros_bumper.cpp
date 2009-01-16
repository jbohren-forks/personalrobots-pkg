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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include <gazebo_plugin/ros_bumper.h>

#include "gazebo/Global.hh"
#include "gazebo/XMLConfig.hh"
#include "ContactSensor.hh"
#include "gazebo/World.hh"
#include "gazebo/gazebo.h"
#include "gazebo/GazeboError.hh"
#include "gazebo/ControllerFactory.hh"
#include "gazebo/Simulator.hh"

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_bumper", RosBumper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosBumper::RosBumper(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<ContactSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Bumper controller requires a Contact Sensor as its parent");

  Param::Begin(&this->parameters);
  this->bumperTopicNameP = new ParamT<std::string>("bumperTopicName", "bumper", 0);
  Param::End();

  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode = new ros::Node("ros_gazebo",ros::Node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in bumper \n");
  }

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosBumper::~RosBumper()
{
  delete this->bumperTopicNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosBumper::LoadChild(XMLConfigNode *node)
{
  this->bumperTopicNameP->Load(node);
  this->bumperTopicName = this->bumperTopicNameP->GetValue();
  std::cout << " publishing contact/collisions to topic name: " << this->bumperTopicName << std::endl;

  rosnode->advertise<std_msgs::String>(this->bumperTopicName,100);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosBumper::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosBumper::UpdateChild()
{
  boost::mutex::scoped_lock sclock(this->lock);

  unsigned int num_contacts = this->myParent->GetContactCount();

  std::string my_geom_name;
  int i_hit_geom;
  double when_i_hit;
  std::string geom_i_hit;;

  for (unsigned int i=0; i < num_contacts; i++)
  {
    my_geom_name = this->myParent->GetGeomName(i); 
    i_hit_geom = this->myParent->GetContactState(i); 
    when_i_hit= this->myParent->GetContactTime(i); 
    geom_i_hit = this->myParent->GetContactGeomName(i); 
    if (i_hit_geom == 1)
    {
      std::ostringstream stream;
      stream    << "touched!    i:" << i
                << "      my geom:" << my_geom_name
                << "   other geom:" << geom_i_hit
                << "         time:" << when_i_hit    << std::endl;
      //std::cout << stream.str();
      this->bumperMsg.data = stream.str();
      rosnode->publish(this->bumperTopicName,this->bumperMsg);
    }
  }

  this->myParent->ResetContactStates();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosBumper::FiniChild()
{
}
