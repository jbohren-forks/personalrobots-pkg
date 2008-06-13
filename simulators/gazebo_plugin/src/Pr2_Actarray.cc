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
// Constructor
Pr2_Actarray::Pr2_Actarray(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("Pr2_Actarray controller requires a Model as its parent");

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pr2_Actarray::~Pr2_Actarray()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Pr2_Actarray::LoadChild(XMLConfigNode *node)
{
   XMLConfigNode *jNode;
   int i =0;
   this->myIface = dynamic_cast<PR2ArrayIface*>(this->ifaces[0]);

   if (!this->myIface)
      gzthrow("Pr2_Actarray controller requires a Actarray Iface");


   for (i=0, jNode = node->GetChild("joint"); jNode; i++)
   {
      std::string name = jNode->GetString("name","",1);

      this->joints[i] = dynamic_cast<Joint*>(this->myParent->GetJoint(name));
      if(this->joints[i]->GetType() == Joint::SLIDER)
      {
         SliderJoint *sjoint = dynamic_cast<SliderJoint*>(this->joints[i]);

         this->joints[i] = dynamic_cast<SliderJoint*>(this->myParent->GetJoint(name));
         this->myIface->data->actuators[i].actualPosition = sjoint->GetPosition();
         this->myIface->data->actuators[i].actualSpeed = sjoint->GetPositionRate();
         this->myIface->data->actuators[i].actualEffectorForce = 0.0;
         this->myIface->data->actuators[i].jointType = PR2::PRISMATIC;
      }
      else
      {
         HingeJoint *hjoint = dynamic_cast<HingeJoint*>(this->joints[i]);

         this->joints[i] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(name));
         this->myIface->data->actuators[i].actualPosition = hjoint->GetAngle();
         this->myIface->data->actuators[i].actualSpeed = hjoint->GetAngleRate();
         this->myIface->data->actuators[i].actualEffectorForce = 0.0; //this must be modified using the JointFeedback struct at some point
         this->myIface->data->actuators[i].jointType = PR2::ROTARY;         

      }

      this->myIface->data->actuators[i].saturationTorque =  jNode->GetDouble("saturationTorque",0.0,1);
      this->myIface->data->actuators[i].pGain =  jNode->GetDouble("pGain",0.0,1);
      this->myIface->data->actuators[i].dGain =  jNode->GetDouble("iGain",0.0,1);
      this->myIface->data->actuators[i].dGain =  jNode->GetDouble("dGain",0.0,1);

      this->myIface->data->actuators[i].timestamp = Simulator::Instance()->GetSimTime();
      this->myIface->data->actuators[i].controlMode = PR2::PD_CONTROL;

      jNode = jNode->GetNext("joint");
   }
   this->numJoints = i;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Pr2_Actarray::InitChild()
{
   for (int i=0; i < this->numJoints; i++)
   {
         this->joints[i]->SetParam( dParamVel, 0.0);
         this->joints[i]->SetParam( dParamFMax, this->myIface->data->actuators[i].saturationTorque );
   }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Pr2_Actarray::UpdateChild(UpdateParams &params)
{
   float positionError, speedError;
   float hiStop, loStop;

   this->myIface->Lock(1);
   this->myIface->data->head.time = Simulator::Instance()->GetSimTime();

   this->myIface->data->actuators_count = this->numJoints;

   //printf("numJoints: %d\n",this->numJoints);
   for (int i=0; i < this->numJoints; i++)
   {
      if(this->joints[i]->GetType() == Joint::SLIDER)
      {
         SliderJoint *sjoint = dynamic_cast<SliderJoint*>(this->joints[i]);
         double cmdPosition = this->myIface->data->actuators[i].cmdPosition;
         double cmdSpeed = this->myIface->data->actuators[i].cmdSpeed;

         switch(this->myIface->data->actuators[i].controlMode)
         {
            case PR2::TORQUE_CONTROL :
               // No fancy controller, just pass the commanded torque/force in (we are not modeling the motors for now)
               sjoint->SetSliderForce(this->myIface->data->actuators[i].cmdEffectorForce);
               break;
            case PR2::PD_CONTROL :
               if (cmdPosition > sjoint->GetHighStop())
                  cmdPosition = sjoint->GetHighStop();
               else if (cmdPosition < sjoint->GetLowStop())
                  cmdPosition = sjoint->GetLowStop();

               positionError = cmdPosition - sjoint->GetPosition();
               speedError = cmdSpeed - sjoint->GetPositionRate();

               if (fabs(positionError) > 0.01)
               {
                  sjoint->SetParam( dParamVel, this->myIface->data->actuators[i].pGain * positionError + this->myIface->data->actuators[i].dGain * speedError);
                  sjoint->SetParam( dParamFMax, this->myIface->data->actuators[i].saturationTorque );
               }
               // printf("SLIDER:: pErr: %f, pGain: %f, dGain: %f, sT: %f\n",positionError,this->myIface->data->actuators[i].pGain,this->myIface->data->actuators[i].dGain,this->myIface->data->actuators[i].saturationTorque);            
               break;
            case PR2::SPEED_CONTROL :
                  sjoint->SetParam( dParamVel, cmdSpeed);
                  sjoint->SetParam( dParamFMax,this->myIface->data->actuators[i].saturationTorque );
            break;

            default:
               break;
         }

         this->myIface->data->actuators[i].actualPosition = sjoint->GetPosition();
         this->myIface->data->actuators[i].actualSpeed = sjoint->GetPositionRate();
         this->myIface->data->actuators[i].actualEffectorForce = 0.0; //this must be modified using the JointFeedback struct at some point
      }
      else
      {
         HingeJoint *hjoint = dynamic_cast<HingeJoint*>(this->joints[i]);
         double cmdPosition = this->myIface->data->actuators[i].cmdPosition;
         double cmdSpeed = this->myIface->data->actuators[i].cmdSpeed;

         switch(this->myIface->data->actuators[i].controlMode)
         {
            case PR2::TORQUE_CONTROL:
               // No fancy controller, just pass the commanded torque/force in (we are not modeling the motors for now)
               hjoint->SetTorque(this->myIface->data->actuators[i].cmdEffectorForce);
               break;
            case PR2::PD_CONTROL:
               if (cmdPosition > hjoint->GetHighStop())
                  cmdPosition = hjoint->GetHighStop();
               else if (cmdPosition < hjoint->GetLowStop())
                  cmdPosition = hjoint->GetLowStop();

               positionError = ModNPi2Pi(cmdPosition - hjoint->GetAngle());
               speedError = cmdSpeed - hjoint->GetAngleRate();

//               printf("ROTARY:: pErr: %f, cmd: %f, actual: %f, pGain: %f, dGain: %f, sT: %f\n",positionError,cmdPosition, hjoint->GetAngle(),this->myIface->data->actuators[i].pGain,this->myIface->data->actuators[i].dGain,this->myIface->data->actuators[i].saturationTorque);
               if (fabs(positionError) > 0.01)
               {
                  hjoint->SetParam( dParamVel, this->myIface->data->actuators[i].pGain * positionError  + this->myIface->data->actuators[i].dGain * speedError);
                  hjoint->SetParam( dParamFMax,this->myIface->data->actuators[i].saturationTorque );
               }
               break;
            case PR2::SPEED_CONTROL:
                  hjoint->SetParam( dParamVel, cmdSpeed);
                  hjoint->SetParam( dParamFMax, this->myIface->data->actuators[i].saturationTorque );
            break;
            default:
               break;
         }
         this->myIface->data->actuators[i].actualPosition = hjoint->GetAngle();
         this->myIface->data->actuators[i].actualSpeed = hjoint->GetAngleRate();
         this->myIface->data->actuators[i].actualEffectorForce = 0.0; //this must be modified using the JointFeedback struct at some point
      }

   }

   this->myIface->data->new_cmd = 0;
   this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Pr2_Actarray::FiniChild()
{
}
