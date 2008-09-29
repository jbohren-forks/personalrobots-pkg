/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GAZEBO_BATTERY_H
#define GAZEBO_BATTERY_H

#include <vector>
#include <map>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include "tinyxml/tinyxml.h"


namespace gazebo
{
class XMLConfigNode;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup gazebo_battery GazeboBattery class

  \brief GazeboBattery Plugin
  
  This plugin simulates battery usage.
  GazeboBattery requires model as its parent.

  \verbatim
  <model:physical name="ray_model">
    <!-- GazeboBattery -->
    <controller:gazebo_battery name="gazebo_battery" plugin="libgazebo_battery.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <interface:audio name="gazebo_battery_dummy_iface" />
    </controller:gazebo_battery>
  </model:phyiscal>
  \endverbatim
 
\{


*/

/**
 * \brief Battery simulation
 *   \li starts a ROS node if none exists
 * .
 *
**/


class GazeboBattery : public gazebo::Controller
{
public:
  GazeboBattery(Entity *parent);
  virtual ~GazeboBattery();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;

  TiXmlDocument config_;

};

/** \} */
/// @}

}

#endif

