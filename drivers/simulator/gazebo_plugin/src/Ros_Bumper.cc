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

#include "Global.hh"
#include "XMLConfig.hh"
#include "ContactSensor.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Simulator.hh"
#include "gazebo_plugin/Ros_Bumper.hh"

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_bumper", Ros_Bumper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Ros_Bumper::Ros_Bumper(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<ContactSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Bumper controller requires a Contact Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Ros_Bumper::~Ros_Bumper()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Ros_Bumper::LoadChild(XMLConfigNode *node)
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Ros_Bumper::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Ros_Bumper::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Ros_Bumper::FiniChild()
{
}
