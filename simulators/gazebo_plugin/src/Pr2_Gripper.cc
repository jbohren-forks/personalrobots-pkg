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
 * Desc: Position2d controller for a Pr2.
 * Author: John Hsu
 * Date: 01 Jun 2008
 * SVN info: $Id$
 */

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo_plugin/Pr2_Gripper.hh>

using namespace gazebo;
using namespace PR2;

GZ_REGISTER_DYNAMIC_CONTROLLER("pr2_gripper", Pr2_Gripper);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pr2_Gripper::Pr2_Gripper(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Pr2_Gripper controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pr2_Gripper::~Pr2_Gripper()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Pr2_Gripper::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PR2GripperIface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("Pr2_Gripper controller requires a PR2GripperIface");

  std::string leftJointName                       = node->GetString("leftJoint"   , "", 1);
  std::string rightJointName                      = node->GetString("rightJoint"  , "", 1);
  this->myIface->data->gripperForce               = node->GetDouble("gripperForce", 0.0, 1);
  this->myIface->data->pGain                      = node->GetDouble("pGain"       , 1.0, 1);
  this->myIface->data->iGain                      = node->GetDouble("iGain"       , 0.0, 1);
  this->myIface->data->dGain                      = node->GetDouble("dGain"       , 0.0, 1);

  this->joints[LEFT] = dynamic_cast<SliderJoint*>(this->myParent->GetJoint(leftJointName));
  this->joints[RIGHT] = dynamic_cast<SliderJoint*>(this->myParent->GetJoint(rightJointName));

  this->myIface->data->actualFingerPosition[LEFT]       = this->joints[LEFT]->GetPosition();
  this->myIface->data->actualFingerPositionRate[LEFT]   = this->joints[LEFT]->GetPositionRate();
  this->myIface->data->actualFingerPosition[RIGHT]      = this->joints[RIGHT]->GetPosition();
  this->myIface->data->actualFingerPositionRate[RIGHT]  = this->joints[RIGHT]->GetPositionRate();

  if (!this->myIface->data->gripperForce)
    gzthrow("couldn't get gripperForce");

  if (!this->joints[LEFT])
    gzthrow("couldn't get left slider joint");

  if (!this->joints[RIGHT])
    gzthrow("couldn't get right slider joint");

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Pr2_Gripper::InitChild()
{
  this->joints[LEFT]->SetParam( dParamVel,0.0);
  this->joints[LEFT]->SetParam( dParamFMax,this->myIface->data->gripperForce );
  this->joints[RIGHT]->SetParam( dParamVel,0.0);
  this->joints[RIGHT]->SetParam( dParamFMax,this->myIface->data->gripperForce );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Pr2_Gripper::UpdateChild(UpdateParams &params)
{
  float cmdPosition[2],positionError[2],positionRateError[2];
  float cmdPositionRate;
  float cmdForce;

  this->myIface->Lock(1);
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime();

  switch (this->myIface->data->cmd)
  {
      case GAZEBO_PR2GRIPPER_CMD_OPEN:
          // TODO: same for OPEN and CLOSE right now,
          //       distinguish between the two by unidirectional forces?
          // TODO: move preset of 0.015 due to geometry of the robot gripper into PR2.hh.
          cmdPosition[LEFT]  =  0.015 - 0.5*this->myIface->data->cmdGap;
          cmdPosition[RIGHT] = -0.015 + 0.5*this->myIface->data->cmdGap;

          std::cout << " left hi " << this->joints[LEFT]->GetHighStop()
                    <<      " lo " << this->joints[LEFT]->GetLowStop()
                    << " rght hi " << this->joints[RIGHT]->GetHighStop()
                    <<      " lo " << this->joints[RIGHT]->GetLowStop() << std::endl;

          // limit by specified stops
          if (cmdPosition[LEFT] > this->joints[LEFT]->GetHighStop())
              cmdPosition[LEFT] = this->joints[LEFT]->GetHighStop();
          else if (cmdPosition[LEFT] < this->joints[LEFT]->GetLowStop())
              cmdPosition[LEFT] = this->joints[LEFT]->GetLowStop();
          if (cmdPosition[RIGHT] > this->joints[RIGHT]->GetHighStop())
              cmdPosition[RIGHT] = this->joints[RIGHT]->GetHighStop();
          else if (cmdPosition[RIGHT] < this->joints[RIGHT]->GetLowStop())
              cmdPosition[RIGHT] = this->joints[RIGHT]->GetLowStop();

          // get rate command TODO: limit max rate?
          cmdPositionRate = this->myIface->data->cmdPositionRate;

          // get force command
          cmdForce = this->myIface->data->cmdForce;
          if (cmdForce > this->myIface->data->gripperForce)
              cmdForce = this->myIface->data->gripperForce;

          // TODO: send joint force command for torque control mode
          //this->joints[LEFT]->SetSliderForce(cmdForce);

          // check position error
          positionError[LEFT]       = cmdPosition[LEFT]  - this->joints[LEFT] ->GetPosition();
          positionError[RIGHT]      = cmdPosition[RIGHT] - this->joints[RIGHT]->GetPosition();
          positionRateError[LEFT]   = cmdPositionRate    - this->joints[LEFT] ->GetPositionRate();
          positionRateError[RIGHT]  = cmdPositionRate    - this->joints[RIGHT]->GetPositionRate();
          std::cout << " opening " << " left cmd " << cmdPosition[LEFT] << " jnt " << this->joints[LEFT] ->GetPosition() << " err " << positionError[LEFT]  << std::endl;
          std::cout << " opening " << " rght cmd " << cmdPosition[RIGHT]<< " jnt " << this->joints[RIGHT]->GetPosition() << " err " << positionError[RIGHT] << std::endl;
          std::cout << " opening rate " << positionRateError[LEFT]  << std::endl;
          std::cout << " opening rate " << positionRateError[RIGHT] << std::endl;
          // send joint position control command
          //if (fabs(positionError[LEFT] ) > 0.01)
          {
            float tmpPosition =  this->myIface->data->pGain * positionError[LEFT]
                                +this->myIface->data->dGain * positionRateError[LEFT] ;
            float tmpForce    =  (tmpPosition > 0.0) ? cmdForce : -cmdForce ;
            this->joints[LEFT] ->SetParam( dParamVel , tmpPosition );
            this->joints[LEFT] ->SetParam( dParamFMax, tmpForce  );
          }
          //if (fabs(positionError[RIGHT]) > 0.01)
          {
            float tmpPosition =  this->myIface->data->pGain * positionError[RIGHT]
                                +this->myIface->data->dGain * positionRateError[RIGHT] ;
            float tmpForce    =  (tmpPosition > 0.0) ? cmdForce : -cmdForce ;
            this->joints[RIGHT]->SetParam( dParamVel , tmpPosition );
            this->joints[RIGHT]->SetParam( dParamFMax, tmpForce );
          }

          break;
      case GAZEBO_PR2GRIPPER_CMD_CLOSE:
          // TODO: same for OPEN and CLOSE right now,
          //       distinguish between the two by unidirectional forces?
          // TODO: move preset of 0.015 due to geometry of the robot gripper into PR2.hh.
          cmdPosition[LEFT]  =  0.015 - 0.5*this->myIface->data->cmdGap;
          cmdPosition[RIGHT] = -0.015 + 0.5*this->myIface->data->cmdGap;

          // limit by specified stops
          if (cmdPosition[LEFT] > this->joints[LEFT]->GetHighStop())
              cmdPosition[LEFT] = this->joints[LEFT]->GetHighStop();
          else if (cmdPosition[LEFT] < this->joints[LEFT]->GetLowStop())
              cmdPosition[LEFT] = this->joints[LEFT]->GetLowStop();
          if (cmdPosition[RIGHT] > this->joints[RIGHT]->GetHighStop())
              cmdPosition[RIGHT] = this->joints[RIGHT]->GetHighStop();
          else if (cmdPosition[RIGHT] < this->joints[RIGHT]->GetLowStop())
              cmdPosition[RIGHT] = this->joints[RIGHT]->GetLowStop();

          // get rate command TODO: limit max rate?
          cmdPositionRate = this->myIface->data->cmdPositionRate;

          // get force command
          cmdForce = this->myIface->data->cmdForce;
          if (cmdForce > this->myIface->data->gripperForce)
              cmdForce = this->myIface->data->gripperForce;

          // TODO: send joint force command for torque control mode
          //this->joints[LEFT]->SetSliderForce(cmdForce);

          // check position error
          positionError[LEFT]       = cmdPosition[LEFT]  - this->joints[LEFT] ->GetPosition();
          positionError[RIGHT]      = cmdPosition[RIGHT] - this->joints[RIGHT]->GetPosition();
          positionRateError[LEFT]   = cmdPositionRate    - this->joints[LEFT] ->GetPositionRate();
          positionRateError[RIGHT]  = cmdPositionRate    - this->joints[RIGHT]->GetPositionRate();
          std::cout << " closing " << positionError[LEFT]  << " "
                                   << positionError[RIGHT] << " "
                                   << positionRateError[LEFT]  << " "
                                   << positionRateError[RIGHT] << std::endl;
          // send joint position control command
          //if (fabs(positionError[LEFT] ) > 0.01)
          {
            float tmpPosition =  this->myIface->data->pGain * positionError[LEFT]
                                +this->myIface->data->dGain * positionRateError[LEFT] ;
            float tmpForce    =  (tmpPosition > 0.0) ? cmdForce : -cmdForce ;
            this->joints[LEFT] ->SetParam( dParamVel , tmpPosition );
            this->joints[LEFT] ->SetParam( dParamFMax, tmpForce  );
          }
          //if (fabs(positionError[RIGHT]) > 0.01)
          {
            float tmpPosition =  this->myIface->data->pGain * positionError[RIGHT]
                                +this->myIface->data->dGain * positionRateError[RIGHT] ;
            float tmpForce    =  (tmpPosition > 0.0) ? cmdForce : -cmdForce ;
            this->joints[RIGHT]->SetParam( dParamVel , tmpPosition );
            this->joints[RIGHT]->SetParam( dParamFMax, tmpForce );
          }

          break;
      case GAZEBO_PR2GRIPPER_CMD_STOP:
          this->joints[LEFT] ->SetParam( dParamVel , this->joints[LEFT] ->GetPosition() );
          this->joints[LEFT] ->SetParam( dParamFMax, 0. );
          this->joints[RIGHT]->SetParam( dParamVel , this->joints[RIGHT]->GetPosition() );
          this->joints[RIGHT]->SetParam( dParamFMax, 0. );
          break;
      case GAZEBO_PR2GRIPPER_CMD_STORE:
          break;
      case GAZEBO_PR2GRIPPER_CMD_RETRIEVE:
          break;
      default:
          break;
  }


  this->myIface->data->actualFingerPosition[LEFT]       = this->joints[LEFT]->GetPosition();
  this->myIface->data->actualFingerPositionRate[LEFT]   = this->joints[LEFT]->GetPositionRate();
  this->myIface->data->actualFingerPosition[RIGHT]      = this->joints[RIGHT]->GetPosition();
  this->myIface->data->actualFingerPositionRate[RIGHT]  = this->joints[RIGHT]->GetPositionRate();

  this->myIface->Unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Pr2_Gripper::FiniChild()
{
}
