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

#ifndef GAZEBO_ACTUATORS_H
#define GAZEBO_ACTUATORS_H

#include <vector>
#include <map>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include "hardware_interface/hardware_interface.h"
#include "mechanism_control/mechanism_control.h"
#include "mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"


namespace gazebo
{
class HingeJoint;
class PositionIface;
class XMLConfigNode;

/**********************************************************/
/*! \class GazeboActuators
    This class implements a plugin for the
    \ref{mechanism::MechanismControl} mechanisms controls class
*/
/**********************************************************/
class GazeboActuators : public gazebo::Controller
{
public:
  GazeboActuators(Entity *parent);
  virtual ~GazeboActuators();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;
  HardwareInterface hw_;
  MechanismControl mc_;
  MechanismControlNode mcn_;

  TiXmlDocument config_;

  // The fake model helps Gazebo run the transmissions backwards, so
  // that it can figure out what its joints should do based on the
  // actuator values.
  // TODO mechanism::Robot fake_model_;
  mechanism::RobotState *fake_state_;
  std::vector<gazebo::Joint*>  joints_;

  // added for joint damping coefficients
  std::vector<double>          joints_damping_;
  std::map<std::string,double> joints_damping_map_;

  /*
   * \brief read pr2.xml for actuators, and pass tinyxml node to mechanism control node's initXml.
   */
  void ReadPr2Xml(XMLConfigNode *node);

  /*
   * \brief read gazebo_joints.xml for joint damping and additional simulation parameters for joints
   */
  void ReadGazeboPhysics(XMLConfigNode *node);
};

}

#endif

