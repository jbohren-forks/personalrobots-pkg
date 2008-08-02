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
#include <fstream>
#include <iostream>

#include <gazebo_plugin/Pr2_Actarray.hh>
#include <math.h>

#include <unistd.h>

using namespace gazebo;
using namespace PR2;
using namespace controller;


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
   int count =0;
   for (jNode = node->GetChild("joint"); jNode ;)
   {
      // add each child
      //actuators.AddJoint();

      // get name of each child, e.g. front_left_caster_steer
      actarrayName[count] = jNode->GetString("name","",1);

      // get type of each child, only check for special case for grippers for now.  All others ignored. (TODO)
      actarrayType[count] = jNode->GetString("type","default",0);  // FIXME: not mandatory for now

      if (actarrayType[count]=="gripper_special") // if this is a gripper, do something about it
      {
        std::cout << " gripper_special joint type " << actarrayType[count] << " " << jNode->GetString("type","default",0) << std::endl;
        // get joints names from xml fields
        finger_l_name     [count] = jNode->GetString("left_proximal","",1);
        finger_tip_l_name [count] = jNode->GetString("left_distal","",1);
        finger_r_name     [count] = jNode->GetString("right_proximal","",1);
        finger_tip_r_name [count] = jNode->GetString("right_distal","",1);

        // get the joints from parent, store the pointers to the joints locally in this class
        finger_l_joint    [count] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(finger_l_name    [count]));
        finger_tip_l_joint[count] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(finger_tip_l_name[count]));
        finger_r_joint    [count] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(finger_r_name    [count]));
        finger_tip_r_joint[count] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(finger_tip_r_name[count]));

        // all four joints controlled by a single iface settings value
        // IMPORTANT: in Iface, we'll use position as the gap command
        // IMPORTANT: in Iface, if in torque control mode, pass through torque to all joints with the right sign
        // given a single gap command, we can find out what the joint angles are
        // FIXME: assuming for now that all joints are hinges

        // initialize some variables in gazebo.h for PR2, FIXME: re-evaluate, this is probably not necessary
        //this->myIface->data->actuators[count].actualPosition      = finger_l_joint[count]->GetAngle(); //TODO: get gap from joint positions
        //this->myIface->data->actuators[count].actualSpeed         = finger_l_joint[count]->GetAngleRate(); //TODO: get gap rate?
        //this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
        this->myIface->data->actuators[count].jointType           = PR2::GRIPPER;         

        // create new pid for each finger for PD_CONTROL
        this->finger_l_pids    [count] = new Pid();
        this->finger_r_pids    [count] = new Pid();
        this->finger_tip_l_pids[count] = new Pid();
        this->finger_tip_r_pids[count] = new Pid();
      }
      else // if this is not a gripper, process them as sliders and hinges
      {
        std::cout << " default joint type " << actarrayType[count] << " " << jNode->GetString("type","default",0) << std::endl;
        // dynamic cast for type checking
        Joint* tmpJoint = dynamic_cast<Joint*>(this->myParent->GetJoint(actarrayName[count]));

        // check what type of joint this is
        switch(tmpJoint->GetType())
        {
          case Joint::SLIDER:
            // save joint in list
            this->joints[count] = tmpJoint;
            // set joint properties
            sjoint = dynamic_cast<SliderJoint*>( tmpJoint );
            // initialize some variables in gazebo.h for PR2, FIXME: re-evaluate, this is probably not necessary
            this->myIface->data->actuators[count].actualPosition      = sjoint->GetPosition();
            this->myIface->data->actuators[count].actualSpeed         = sjoint->GetPositionRate();
            this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
            this->myIface->data->actuators[count].jointType           = PR2::PRISMATIC;
            break;
          case Joint::HINGE:
            // save joint in list
            this->joints[count] = tmpJoint;
            // set joint properties
            hjoint = dynamic_cast<HingeJoint*>( tmpJoint );
            // initialize some variables in gazebo.h for PR2, FIXME: re-evaluate, this is probably not necessary
            this->myIface->data->actuators[count].actualPosition      = hjoint->GetAngle();
            this->myIface->data->actuators[count].actualSpeed         = hjoint->GetAngleRate();
            this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
            this->myIface->data->actuators[count].jointType           = PR2::ROTARY;         
            break;
          case Joint::HINGE2:
          case Joint::BALL:
          case Joint::UNIVERSAL:
            gzthrow("Pr2_Actarray joint need to be either slider or hinge for now.  Add support in Pr2_Actarray or talk to ");
            break;
        }

      }

      // get additional information about this actuator
      this->myIface->data->actuators[count].saturationTorque   =  jNode->GetDouble("saturationTorque",0.0,1);
      this->myIface->data->actuators[count].pGain              =  jNode->GetDouble("pGain" ,0.0,1);
      this->myIface->data->actuators[count].iGain              =  jNode->GetDouble("iGain" ,0.0,1);
      this->myIface->data->actuators[count].dGain              =  jNode->GetDouble("dGain" ,0.0,1);
      this->myIface->data->actuators[count].iClamp             =  jNode->GetDouble("iClamp",0.0,1);
      std::string tmpControlMode                               =  jNode->GetString("controlMode","PD_CONTROL",0);
      this->myIface->data->actuators[count].dampingCoefficient =  jNode->GetDouble("explicitDampingCoefficient",0.0,0);

      // init a new pid for this joint
      this->pids[count] = new Pid();

      // get time
      this->myIface->data->actuators[count].timestamp = Simulator::Instance()->GetSimTime();
      // set default control mode to:
      if (tmpControlMode == "TORQUE_CONTROL")
          this->myIface->data->actuators[count].controlMode = PR2::TORQUE_CONTROL;
      else if (tmpControlMode == "PD_TORQUE_CONTROL")
          this->myIface->data->actuators[count].controlMode = PR2::PD_TORQUE_CONTROL;
      else if (tmpControlMode == "PD_CONTROL")
          this->myIface->data->actuators[count].controlMode = PR2::PD_CONTROL;
      else
          this->myIface->data->actuators[count].controlMode = PR2::PD_CONTROL;

      jNode = jNode->GetNext("joint");
      count++;
   }
   this->num_joints = count;
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
      if (actarrayType[count]=="gripper_special") // if this is a gripper, do something about it
      {
        // FIXME: temporary hack, initialize finger pid stuff, this should be based on transmissions
        this->finger_l_pids[count]     -> InitPid ( 1.0, 0.01, 0.0, 0.2, -0.2 );
        this->finger_r_pids[count]     -> InitPid ( 1.0, 0.01, 0.0, 0.2, -0.2 );
        this->finger_tip_l_pids[count] -> InitPid ( 1.0, 0.01, 0.0, 0.2, -0.2 );
        this->finger_tip_r_pids[count] -> InitPid ( 1.0, 0.01, 0.0, 0.2, -0.2 );
        this->finger_l_joint[count]     -> SetParam( dParamVel, 0 );
        this->finger_r_joint[count]     -> SetParam( dParamVel, 0 );
        this->finger_tip_l_joint[count] -> SetParam( dParamVel, 0 );
        this->finger_tip_r_joint[count] -> SetParam( dParamVel, 0 );
        this->finger_l_joint[count]     -> SetParam( dParamFMax, 0 );
        this->finger_r_joint[count]     -> SetParam( dParamFMax, 0 );
        this->finger_tip_l_joint[count] -> SetParam( dParamFMax, 0 );
        this->finger_tip_r_joint[count] -> SetParam( dParamFMax, 0 );
      }
      else
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
        //this->joints[count]->SetParam( dParamVel , this->pids[count]->GetCurrentCmd());
        //this->joints[count]->SetParam( dParamFMax, this->myIface->data->actuators[count].saturationTorque );
        this->joints[count]->SetParam( dParamVel , 0);
        this->joints[count]->SetParam( dParamFMax, 0);
      }
   }
   lastTime = Simulator::Instance()->GetSimTime();
}


#ifdef ADVAIT
void Pr2_Actarray::UpdateChild()
{
  HingeJoint *hjoint;
  double cmdPosition;
  double currentTime;

  double curr_ang;
  double mass, length;
  double g;
  double gravity_torque;
  static double error, error_prev=-200;
  double kp,kd;
  double apply_torque;

  this->myIface->Lock(1);

  currentTime = Simulator::Instance()->GetSimTime();
  this->myIface->data->head.time = currentTime;
  this->myIface->data->actuators_count = this->num_joints;

  //--------- loop through all the controllable dof's in this interface ----------
  for (int count = 0; count < this->num_joints; count++)
  {
    switch(this->joints[count]->GetType())
    {
      case Joint::HINGE:
        hjoint = dynamic_cast<HingeJoint*>(this->joints[count]);
        cmdPosition = this->myIface->data->actuators[count].cmdPosition;
        cmdPosition = DTOR(60.0);
        curr_ang = hjoint->GetAngle();
        mass = 5;
        length = 0.25;
        g = 9.8;
        gravity_torque = -mass*g*cos(curr_ang)*length;
        error = cmdPosition-curr_ang;
        if (error_prev == -200)
          error_prev = error;

        kp = 1.;
        kd = 50.0;
        printf("---------------\n");
        printf("error: %f\t error_prev: %f\n", RTOD(error), RTOD(error_prev));
        printf("kp term: %.3f\tkd term: %.3f\t", kp*error, kd*(error-error_prev));
        apply_torque = -gravity_torque + kp*error + kd*(error-error_prev);
        error_prev = error;
        printf("error: %.2f\tapplied torque:%f\n", RTOD(error), apply_torque);
        hjoint->SetTorque(apply_torque);

        this->myIface->data->actuators[count].actualPosition      = hjoint->GetAngle();
        this->myIface->data->actuators[count].actualSpeed         = hjoint->GetAngleRate();
        this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point
        break;

      case Joint::SLIDER:
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

#else
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
void Pr2_Actarray::UpdateChild()
{
   float positionError, speedError;
   HingeJoint *hjoint;
   SliderJoint *sjoint;
   double cmdPosition, cmdSpeed;
   double currentTime;
   double currentCmd;
   double currentRate;
   double currentAngle;

   // TODO: EXPERIMENTAL: add explicit damping
   double dampForce;

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
      if (actarrayType[count]=="gripper_special") // if this is a gripper, do something about it
      {
        // get commands
        cmdPosition = this->myIface->data->actuators[count].cmdPosition;
        cmdSpeed    = this->myIface->data->actuators[count].cmdSpeed;
        // translate command position into joint angle
        // command position is in radians
        static double angleGapRatio =  0.2 / M_PI * 4.0;
        double cmdGripperJointAngle = cmdPosition / angleGapRatio;  // FIXME: TEMP HACK for transmission, 45 degrees with gripper gap command of 0.2 (meters)

        // get control mode
        switch(this->myIface->data->actuators[count].controlMode)
        {
            case PR2::TORQUE_CONTROL:
               // EXPERIMENTAL: add explicit damping
               /* ----------- finger_l ---------- */
               currentRate = finger_l_joint[count]->GetAngleRate();
               dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
               dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
               dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
               //printf("Damping f %f v %f\n",dampForce,currentRate);
               //std::cout<<"Force is:"<< this->myIface->data->actuators[count].cmdEffectorForce + dampForce<<std::endl;
               // simply set torque
               finger_l_joint[count]->SetTorque(this->myIface->data->actuators[count].cmdEffectorForce + dampForce);
               //std::cout << count << " " << this->myIface->data->actuators[count].controlMode << std::endl;
               /* ----------- finger_tip_l ---------- */
               currentRate = finger_tip_l_joint[count]->GetAngleRate();
               dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
               dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
               dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
               finger_tip_l_joint[count]->SetTorque(-(this->myIface->data->actuators[count].cmdEffectorForce + dampForce)); // negative enforces parallel
               /* ----------- finger_r ---------- */
               currentRate = finger_r_joint[count]->GetAngleRate();
               dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
               dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
               dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
               finger_r_joint[count]->SetTorque(-(this->myIface->data->actuators[count].cmdEffectorForce + dampForce)); // negative enforces symmetry
                /* ----------- finger_tip_r ---------- */
               currentRate = finger_tip_r_joint[count]->GetAngleRate();
               dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
               dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
               dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
               finger_tip_r_joint[count]->SetTorque(this->myIface->data->actuators[count].cmdEffectorForce + dampForce);
               break;
            case PR2::PD_TORQUE_CONTROL :
               break;
           case PR2::SPEED_TORQUE_CONTROL :
               break;
           case PR2::PD_CONTROL:
               // positive angle = open gripper
               // no negative angle
               /* ----------- finger_l ---------- */
               currentAngle  = finger_l_joint[count]->GetAngle();
               currentRate   = finger_l_joint[count]->GetAngleRate();
               positionError = ModNPi2Pi(currentAngle - cmdGripperJointAngle);
               speedError    = currentRate - cmdSpeed;
               currentCmd    = this->finger_l_pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
               finger_l_joint[count]->SetParam( dParamVel, currentCmd );
               finger_l_joint[count]->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );

               /* ----------- finger_tip_l ---------- */
               currentAngle  = finger_tip_l_joint[count]->GetAngle();
               currentRate   = finger_tip_l_joint[count]->GetAngleRate();
               positionError = ModNPi2Pi(currentAngle + cmdGripperJointAngle); // negative command angle enforces parallel gripper
               speedError    = currentRate - cmdSpeed;
               currentCmd    = this->finger_tip_l_pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
               finger_tip_l_joint[count]->SetParam( dParamVel, currentCmd );
               finger_tip_l_joint[count]->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );

               /* ----------- finger_r ---------- */
               currentAngle  = finger_r_joint[count]->GetAngle();
               currentRate   = finger_r_joint[count]->GetAngleRate();
               positionError = ModNPi2Pi(currentAngle + cmdGripperJointAngle); // negative command angle enforces symmetric gripper
               speedError    = currentRate - cmdSpeed;
               currentCmd    = this->finger_r_pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
               finger_r_joint[count]->SetParam( dParamVel, currentCmd );
               finger_r_joint[count]->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );

               /* ----------- finger_tip_r ---------- */
               currentAngle  = finger_tip_r_joint[count]->GetAngle();
               currentRate   = finger_tip_r_joint[count]->GetAngleRate();
               positionError = ModNPi2Pi(currentAngle - cmdGripperJointAngle);
               speedError    = currentRate - cmdSpeed;
               currentCmd    = this->finger_tip_r_pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
               finger_tip_r_joint[count]->SetParam( dParamVel, currentCmd );
               finger_tip_r_joint[count]->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );

               break;
           case PR2::SPEED_CONTROL:
               // positive speed = open speed, (since positive angle implies open gripper)
               finger_l_joint[count]->SetParam( dParamVel ,  cmdSpeed);
               finger_l_joint[count]->SetParam( dParamFMax,  this->myIface->data->actuators[count].saturationTorque );
               finger_r_joint[count]->SetParam( dParamVel , -cmdSpeed);
               finger_r_joint[count]->SetParam( dParamFMax,  this->myIface->data->actuators[count].saturationTorque );
               finger_tip_l_joint[count]->SetParam( dParamVel ,  cmdSpeed);
               finger_tip_l_joint[count]->SetParam( dParamFMax,  this->myIface->data->actuators[count].saturationTorque );
               finger_tip_r_joint[count]->SetParam( dParamVel , -cmdSpeed);
               finger_tip_r_joint[count]->SetParam( dParamFMax,  this->myIface->data->actuators[count].saturationTorque );
               break;
           case PR2::DISABLED:
               finger_l_joint[count]->SetParam( dParamFMax, 0);
               finger_l_joint[count]->SetTorque(0); //disable the joint
               finger_r_joint[count]->SetParam( dParamFMax, 0);
               finger_r_joint[count]->SetTorque(0); //disable the joint
               finger_tip_l_joint[count]->SetParam( dParamFMax, 0);
               finger_tip_l_joint[count]->SetTorque(0); //disable the joint
               finger_tip_r_joint[count]->SetParam( dParamFMax, 0);
               finger_tip_r_joint[count]->SetTorque(0); //disable the joint
               break;
           default:
               break;

        }
        this->myIface->data->actuators[count].actualPosition      = finger_l_joint[count]->GetAngle()*angleGapRatio;  // FIXME: TEMP HACK for transmission, 45 degrees with gripper gap command of 0.2 (meters)
        this->myIface->data->actuators[count].actualSpeed         = finger_l_joint[count]->GetAngleRate();
        this->myIface->data->actuators[count].actualEffectorForce = 0.0; //TODO: use JointFeedback struct at some point


      }
      else // if this is not a gripper, process them as sliders and hinges
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
                   case PR2::PD_TORQUE_CONTROL :
                   case PR2::PD_CONTROL : // velocity control
                       //if (cmdPosition > sjoint->GetHighStop())
                       //   cmdPosition = sjoint->GetHighStop();
                       //else if (cmdPosition < sjoint->GetLowStop())
                       //   cmdPosition = sjoint->GetLowStop();
    
                       positionError = sjoint->GetPosition() - cmdPosition; //Error defined as actual - desired
                       speedError    = sjoint->GetPositionRate() - cmdSpeed;
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
                      // TODO: EXPERIMENTAL: add explicit damping
                      currentRate = hjoint->GetAngleRate();
                      dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
                      dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
                      dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
                      //printf("Damping f %f v %f\n",dampForce,currentRate);
                      //std::cout<<"Force is:"<< this->myIface->data->actuators[count].cmdEffectorForce + dampForce<<std::endl;
                      // simply set torque
                      hjoint->SetTorque(this->myIface->data->actuators[count].cmdEffectorForce + dampForce);
                      //std::cout << count << " " << this->myIface->data->actuators[count].controlMode << std::endl;
                      break;
                   case PR2::PD_TORQUE_CONTROL :
                      // TODO: EXPERIMENTAL: add explicit damping
                      currentRate = hjoint->GetAngleRate();
                      dampForce = - this->myIface->data->actuators[count].dampingCoefficient * currentRate;
                      dampForce = (dampForce >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : dampForce;
                      dampForce = (dampForce < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : dampForce;
                      //printf("Damping f %f v %f\n",dampForce,currentRate);

                      currentAngle = hjoint->GetAngle();
                      // No fancy controller, just pass the commanded torque/force in (we are not modeling the motors for now)
                      positionError = ModNPi2Pi( currentAngle - cmdPosition);
                      speedError    =  currentRate- cmdSpeed;
                      currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
                      // if(count==PR2::ARM_R_SHOULDER_PITCH ) std::cout << "hinge err:" << positionError << " cmd: " << currentCmd << std::endl;
                      //Write out data
                      if(count==PR2::ARM_R_SHOULDER_PITCH ) std::cout << currentTime<<" "<<cmdPosition<<" "<<currentAngle<<" "<<positionError<< " "<<currentCmd << std::endl;

                      // limit torque
                      currentCmd = (currentCmd >  100) ?  100: currentCmd;
                      currentCmd = (currentCmd < -100 ) ? -100: currentCmd;
                      //currentCmd = (currentCmd >  this->myIface->data->actuators[count].saturationTorque ) ?  this->myIface->data->actuators[count].saturationTorque : currentCmd;
                      //currentCmd = (currentCmd < -this->myIface->data->actuators[count].saturationTorque ) ? -this->myIface->data->actuators[count].saturationTorque : currentCmd;
                      hjoint->SetParam( dParamFMax, 0);
                      hjoint->SetTorque(currentCmd + dampForce);

                      break;
                  case PR2::SPEED_TORQUE_CONTROL :
                      currentRate = hjoint->GetAngleRate();
                      speedError    = currentRate - cmdSpeed;
                      currentCmd    = this->pids[count]->UpdatePid(speedError, currentTime-this->lastTime);
                      //if(count==PR2::ARM_R_PAN)std::cout<<"Joint:" <<count << " Desired:" << cmdSpeed << " Current speed"<<currentRate<<" Error"<<speedError<<" cmd: " << currentCmd << std::endl;
                      if(count==PR2::ARM_R_ELBOW_PITCH )std::cout<<currentTime<<" "<<currentRate<<" "<<speedError<<" " << currentCmd << std::endl;
                      // limit torque
        
                      currentCmd = (currentCmd >  100) ?  100: currentCmd;
                      currentCmd = (currentCmd < -100 ) ? -100: currentCmd;
    
                      //Needs to be set to 0 1x for every joint
                      hjoint->SetParam( dParamFMax, 0);
                      hjoint->SetTorque(currentCmd);
      

                      break;
                  case PR2::PD_CONTROL:
                      //if (cmdPosition > hjoint->GetHighStop())
                      //   cmdPosition = hjoint->GetHighStop();
                      //else if (cmdPosition < hjoint->GetLowStop())
                      //   cmdPosition = hjoint->GetLowStop();
   
                      currentAngle  = hjoint->GetAngle();
                      currentRate   = hjoint->GetAngleRate();
                      positionError = ModNPi2Pi(currentAngle - cmdPosition);
                      speedError    = currentRate - cmdSpeed;
                      //std::cout << "hinge e:" << speedError << " + " << positionError << std::endl;
                      currentCmd    = this->pids[count]->UpdatePid(positionError + 0.0*speedError, currentTime-this->lastTime);
                      hjoint->SetParam( dParamVel, currentCmd );
                      hjoint->SetParam( dParamFMax,this->myIface->data->actuators[count].saturationTorque );
                      break;
                  case PR2::SPEED_CONTROL:
                      // printf("Hinge Speed Control\n");
                      //std::cout << "wheel drive: " << cmdSpeed << " i " << count << std::endl;
                      hjoint->SetParam( dParamVel, cmdSpeed);
                      hjoint->SetParam( dParamFMax, this->myIface->data->actuators[count].saturationTorque );
                      break;
                  case PR2::DISABLED:
                      hjoint->SetParam( dParamFMax, 0);
                      hjoint->SetTorque(0); //disable the joint
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
        } // end of switch block
      }  // end of if motor block
   } // end of for-count block

   this->myIface->data->new_cmd = 0;
   this->myIface->Unlock();
   this->lastTime = currentTime;
}

#endif

////////////////////////////////////////////////////////////////////////////////
//
// Finalize the controller
//
////////////////////////////////////////////////////////////////////////////////
void Pr2_Actarray::FiniChild()
{
 
}
