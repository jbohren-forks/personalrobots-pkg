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

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <gazebo_plugin/Pr2_Actarray.hh>

using namespace gazebo;
using namespace PR2;

//GZ_REGISTER_STATIC_CONTROLLER("pr2_actarray", Pr2_Actarray);
GZ_REGISTER_DYNAMIC_CONTROLLER("pr2_actarray", Pr2_Actarray);

double ModNPi2Pi(double angle)
{
   double theta = angle - ((int)(angle/(2*M_PI))*2*M_PI);
   //double theta = fmod(angle,2*M_PI);
   double result = theta;

   if (theta > M_PI) 
      result = theta - 2*M_PI;
   if(theta < -M_PI)
      result = theta + 2*M_PI;

   return result;
}

////////////////////////////////////////////////////////////////////////////////
//
// Constructor
//
////////////////////////////////////////////////////////////////////////////////
Pr2_Actarray::Pr2_Actarray(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("Pr2_Actarray controller requires a Model as its parent");

}

////////////////////////////////////////////////////////////////////////////////
//
// Destructor
//
////////////////////////////////////////////////////////////////////////////////
Pr2_Actarray::~Pr2_Actarray()
{
}

////////////////////////////////////////////////////////////////////////////////
//
// Load the controller
//
// loop through all joints
//     check joint type
//         initialize variables
//
////////////////////////////////////////////////////////////////////////////////
void Pr2_Actarray::LoadChild(XMLConfigNode *node)
{
   // gazebo pr2 actarray interface: connect and check
   // this->ifaces[0] is inherited from parent class

   XMLConfigNode *jNode;
   this->myIface = dynamic_cast<PR2ArrayIface*>(this->ifaces[0]);
   if (!this->myIface)
      gzthrow("Pr2_Actarray controller requires a Actarray Iface");

   SliderJoint *sjoint;
   HingeJoint  *hjoint;


   // get children of the actarray, add to actuators object list
   int i =0;
   for (i=0, jNode = node->GetChild("joint"); jNode; i++)
   {
      // add each child
      //actuators.AddJoint();

      // get name of each child, e.g. front_left_caster_steer
      std::string name = jNode->GetString("name","",1);

      // dynamic cast for type checking
      Joint* tmpJoint = dynamic_cast<Joint*>(this->myParent->GetJoint(name));

      // check what type of joint this is
      switch(tmpJoint->GetType())
      {
        case Joint::SLIDER:
          // save joint in list
          this->joints[i] = tmpJoint;
          // set joint properties
          sjoint = dynamic_cast<SliderJoint*>( tmpJoint );
          // initialize some variables in gazebo.h for PR2, FIXME: re-evaluate, this is probably not necessary
          this->myIface->data->actuators[i].actualPosition      = sjoint->GetPosition();
          this->myIface->data->actuators[i].actualSpeed         = sjoint->GetPositionRate();
          this->myIface->data->actuators[i].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
          this->myIface->data->actuators[i].jointType           = PR2::PRISMATIC;
          break;
        case Joint::HINGE:
          // save joint in list
          this->joints[i] = tmpJoint;
          // set joint properties
          hjoint = dynamic_cast<HingeJoint*>( tmpJoint );
          // initialize some variables in gazebo.h for PR2, FIXME: re-evaluate, this is probably not necessary
          this->myIface->data->actuators[i].actualPosition      = hjoint->GetAngle();
          this->myIface->data->actuators[i].actualSpeed         = hjoint->GetAngleRate();
          this->myIface->data->actuators[i].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
          this->myIface->data->actuators[i].jointType           = PR2::ROTARY;         
          break;
        case Joint::HINGE2:
        case Joint::BALL:
        case Joint::UNIVERSAL:
          gzthrow("Pr2_Actarray joint need to be either slider or hinge for now.  Add support in Pr2_Actarray or talk to ");
          break;
      }

      this->myIface->data->actuators[i].saturationTorque =  jNode->GetDouble("saturationTorque",0.0,1);
      this->myIface->data->actuators[i].pGain            =  jNode->GetDouble("pGain" ,0.0,1);
      this->myIface->data->actuators[i].iGain            =  jNode->GetDouble("iGain" ,0.0,1);
      this->myIface->data->actuators[i].dGain            =  jNode->GetDouble("dGain" ,0.0,1);
      this->myIface->data->actuators[i].iClamp           =  jNode->GetDouble("iClamp",0.0,1);
      //this->myIface->data->actuators[i].cmdPosition      =  0.0;
      //this->myIface->data->actuators[i].cmdSpeed         =  0.0;

      // init a new pid for this joint
      this->pids[i] = new Pid();

      // get time
      this->myIface->data->actuators[i].timestamp = Simulator::Instance()->GetSimTime();
      // set default control mode to:
      //this->myIface->data->actuators[i].controlMode = PR2::TORQUE_CONTROL;
      this->myIface->data->actuators[i].controlMode = PR2::PD_CONTROL;

      jNode = jNode->GetNext("joint");
   }
   this->num_joints = i;
}

////////////////////////////////////////////////////////////////////////////////
//
// Initialize the controller
//
// set all joints to zero velocity and saturation torque
//
////////////////////////////////////////////////////////////////////////////////
void Pr2_Actarray::InitChild()
{
   for (int count = 0; count < this->num_joints ; count++)
   {
      // initialize pid stuff
      this->pids[count]->InitPid( this->myIface->data->actuators[count].pGain,
                                  this->myIface->data->actuators[count].iGain,
                                  this->myIface->data->actuators[count].dGain,
                                  this->myIface->data->actuators[count].iClamp,
                                 -this->myIface->data->actuators[count].iClamp
                                );
      this->pids[count]->SetCurrentCmd(0);
      // as a first hack, initialize to zero velocity and saturation torque
      this->joints[count]->SetParam( dParamVel , this->pids[count]->GetCurrentCmd());
      this->joints[count]->SetParam( dParamFMax, this->myIface->data->actuators[count].saturationTorque );
   }
   lastTime = Simulator::Instance()->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// 
// Update the controller
// 
// one step in a PID loop
// 
// ALGORITHM
// =========
// go through all joints,
//     check joint type
//         check control type
//             check error
//             compute and set new command
// 
// ODE I/O
// =======
// all we can set in ode is
//     velocity
//     torque
//
// Control Modes
// =============
// torque control mode
//     set velocity to the right "side" of the current velocity
//     run pid on the torque value
// pd control
//     set velocity based on position error
//     set torque to saturation torque
// velocity control
//     set velocity based on velocity error
//     set torque to saturation torque
//
////////////////////////////////////////////////////////////////////////////////
void Pr2_Actarray::UpdateChild(UpdateParams &params)
{
   float positionError, speedError;
   HingeJoint *hjoint;
   SliderJoint *sjoint;
   double cmdPosition, cmdSpeed;
   double currentTime;
   double currentCmd;

   this->myIface->Lock(1);

   currentTime = Simulator::Instance()->GetSimTime();
   this->myIface->data->head.time = currentTime;

   this->myIface->data->actuators_count = this->num_joints;
   //printf("number of joints: %d\n",this->num_joints);

   //////////////////////////////////////////////////////////////////////
   //
   // LOOP THROUGH ALL THE CONTROLLABLE DOF'S IN THIS INTERFACE
   //
   //////////////////////////////////////////////////////////////////////
   for (int count = 0; count < this->num_joints; count++)
   {
      switch(this->joints[count]->GetType())
      {
         case Joint::SLIDER:
             sjoint = dynamic_cast<SliderJoint*>(this->joints[count]);
             cmdPosition = this->myIface->data->actuators[count].cmdPosition;
             cmdSpeed = this->myIface->data->actuators[count].cmdSpeed;

             switch(this->myIface->data->actuators[count].controlMode)
             {
                case PR2::TORQUE_CONTROL :
                   sjoint->SetSliderForce(this->myIface->data->actuators[count].cmdEffectorForce);
                   break;
                // case PR2::PD_CONTROL1 :
                //    // No fancy controller, just pass the commanded torque/force in (we are not modeling the motors for now)
                //    positionError = cmdPosition - sjoint->GetPosition();
                //    speedError    = cmdSpeed    - sjoint->GetPositionRate();
                //    //std::cout << "slider e:" << speedError << " + " << positionError << std::endl;
                //    currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
                //    currentCmd = (currentCmd >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : currentCmd;
                //    currentCmd = (currentCmd < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : currentCmd;
                //    sjoint->SetSliderForce(currentCmd);
                //    break;
                case PR2::PD_CONTROL : // velocity control
                   //if (cmdPosition > sjoint->GetHighStop())
                   //   cmdPosition = sjoint->GetHighStop();
                   //else if (cmdPosition < sjoint->GetLowStop())
                   //   cmdPosition = sjoint->GetLowStop();

                   positionError = cmdPosition - sjoint->GetPosition();
                   speedError    = cmdSpeed    - sjoint->GetPositionRate();
                   //std::cout << "slider e:" << speedError << " + " << positionError << std::endl;
                   currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);

                   sjoint->SetParam( dParamVel , currentCmd );
                   sjoint->SetParam( dParamFMax, this->myIface->data->actuators[count].saturationTorque );
                   break;
                case PR2::SPEED_CONTROL :
                      sjoint->SetParam( dParamVel, cmdSpeed);
                      sjoint->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );
                break;

                default:
                   break;
             }

             this->myIface->data->actuators[count].actualPosition      = sjoint->GetPosition();
             this->myIface->data->actuators[count].actualSpeed         = sjoint->GetPositionRate();
             this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
             break;

          case Joint::HINGE:
             hjoint = dynamic_cast<HingeJoint*>(this->joints[count]);
             cmdPosition = this->myIface->data->actuators[count].cmdPosition;
             cmdSpeed = this->myIface->data->actuators[count].cmdSpeed;
             switch(this->myIface->data->actuators[count].controlMode)
             {
                case PR2::TORQUE_CONTROL:
									printf("Hinge Torque Control\n");
                   hjoint->SetTorque(this->myIface->data->actuators[count].cmdEffectorForce);
                   //std::cout << count << " " << this->myIface->data->actuators[count].controlMode << std::endl;
                   break;
                // case PR2::PD_CONTROL1 :
                //    // No fancy controller, just pass the commanded torque/force in (we are not modeling the motors for now)
                //    positionError = ModNPi2Pi(cmdPosition - hjoint->GetAngle());
                //    speedError    = cmdSpeed - hjoint->GetAngleRate();
                //    currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
                //    std::cout << "hinge err:" << positionError << " cmd: " << currentCmd << std::endl;
                //    // limit torque
                //    currentCmd = (currentCmd >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : currentCmd;
                //    currentCmd = (currentCmd < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : currentCmd;
                //    hjoint->SetTorque(currentCmd);
                //    break;
                case PR2::PD_CONTROL:
                   //if (cmdPosition > hjoint->GetHighStop())
                   //   cmdPosition = hjoint->GetHighStop();
                   //else if (cmdPosition < hjoint->GetLowStop())
                   //   cmdPosition = hjoint->GetLowStop();

                   positionError = ModNPi2Pi(cmdPosition - hjoint->GetAngle());
                   speedError    = cmdSpeed - hjoint->GetAngleRate();
                   //std::cout << "hinge e:" << speedError << " + " << positionError << std::endl;
                   currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);

                   hjoint->SetParam( dParamVel, currentCmd );
                   hjoint->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );
                   break;
                case PR2::SPEED_CONTROL:
									printf("Hinge Speed Control\n");
                      //std::cout << "wheel drive: " << cmdSpeed << " i " << count << std::endl;
                      hjoint->SetParam( dParamVel, cmdSpeed);
                      hjoint->SetParam( dParamFMax, this->myIface->data->actuators[count].saturationTorque );
                break;
                default:
                   break;
             }
             this->myIface->data->actuators[count].actualPosition      = hjoint->GetAngle();
             this->myIface->data->actuators[count].actualSpeed         = hjoint->GetAngleRate();
             this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
             break;
          case Joint::HINGE2:
          case Joint::BALL:
          case Joint::UNIVERSAL:
             break;
       }
   }

   this->myIface->data->new_cmd = 0;
   this->myIface->Unlock();
   this->lastTime = currentTime;
}

////////////////////////////////////////////////////////////////////////////////
//
// Finalize the controller
//
////////////////////////////////////////////////////////////////////////////////
void Pr2_Actarray::FiniChild()
{
}
