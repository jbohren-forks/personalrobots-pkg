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

#include <gazebo_plugin/test_actuators.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stl_utils/stl_utils.h>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("test_actuators", GazeboActuators);

GazeboActuators::GazeboActuators(Entity *parent)
  : Controller(parent), hw_(0), mc_(&hw_) //, mcn_(&mc_)
{
   this->parent_model_ = dynamic_cast<Model*>(this->parent);

   if (!this->parent_model_)
      gzthrow("GazeboActuators controller requires a Model as its parent");

  rosnode_ = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode_ == NULL)
  {
    // this only works for a single camera.
    ros::init(argc,argv);
    rosnode_ = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in camera \n");
  }
  tfc = new rosTFClient(*rosnode_); //, true, 1 * 1000000000ULL, 0ULL);

  // uses info from wg_robot_description_parser/send.xml
  std::string pr2Content;
  // get pr2.xml for Ioan's parser
  rosnode_->get_param("robotdesc/pr2",pr2Content);
  // parse the big pr2.xml string from ros
  pr2Description.loadString(pr2Content.c_str());
}

GazeboActuators::~GazeboActuators()
{
  deleteElements(&gazebo_joints_);
}

void GazeboActuators::LoadChild(XMLConfigNode *node)
{
  // if we were doing some ros publishing...
  //this->topicName = node->GetString("topicName","default_ros_camera",0); //read from xml file
  //this->frameName = node->GetString("frameName","default_ros_camera",0); //read from xml file
  //std::cout << "================= " << this->topicName << std::endl;
  //rosnode_->advertise<std_msgs::Image>(this->topicName);

  //---------------------------------------------------------------------
  // setup mechanism control - non-gazebo stuff,
  // same as implemented on realtime system
  //
  // for the topside, we need to setup:
  //   robot->joints
  //   actuators
  //   transmissions
  // 
  //---------------------------------------------------------------------
  LoadMC(node);

#if 0
  //---------------------------------------------------------------------
  // setup gazebo related hardware
  // Gazebo gets actuators information
  // reverse transform to joint info and send to simulated joints
  //
  // here, we need to, once again, setup:
  //   transmission
  //     given actuators from mc_
  //   new set up joints, these are the gazebo hardware joints, not the abstract robot->joints
  //
  //
  // one would control robot->joints by controllers, propagate through tranmission to actuators
  // ^^^ top side ^^^
  //
  // vvv bottom side vvv
  // take actuator data and back propagate to joint data, then send to gazebo
  //
  //
  //---------------------------------------------------------------------

  XMLConfigNode *xit;  // XML iterator

  xit = node->GetChild("robot");
  std::cout << " LoadChild test_actuator " <<  xit->GetString("name","",0) << std::endl;

  // Reads the joints out of Gazebo's config.
  std::map<std::string,int> joint_lookup;
  for (xit = node->GetChild("joint"); xit; xit = xit->GetNext("joint"))
  {
    std::string joint_name = xit->GetString("name", "", 1);
    gazebo::Joint *joint = parent_model_->GetJoint(joint_name);
    assert(joint != NULL);
    gazebo_joints_.push_back(joint);
    mech_joints_.push_back(new mechanism::Joint);

    joint_lookup.insert(std::pair<std::string,int>(joint_name, gazebo_joints_.size()-1));
  }

  // Reads the transmission information from the config.
  for (XMLConfigNode *zit = node->GetChild("robot"); zit; zit = zit->GetNext("robot"))
  for (xit = zit->GetChild("transmission"); xit; xit = xit->GetNext("transmission"))
  {
    if (0 == strcmp("SimpleTransmission", xit->GetString("type", "", 0).c_str()))
    {
      // Looks up the joint by name
      XMLConfigNode *joint_node = xit->GetChild("joint");
      assert(joint_node != NULL);
      std::map<std::string,int>::iterator jit = joint_lookup.find(joint_node->GetString("name", "", 1));
      if (jit == joint_lookup.end())
      {
        // TODO: report: Could not find the joint named xxxx
        continue;
      }

      // Creates the actuator
      XMLConfigNode *actuator_node = xit->GetChild("actuator");
      assert(actuator_node != NULL);
      Actuator *actuator = new Actuator;
      hw_.actuators_.push_back(actuator);
      actuator_names_.push_back(actuator_node->GetString("name", "", 1));

      // Creates the transmission
      transmissions_.push_back(
        new mechanism::SimpleTransmission(mech_joints_[jit->second], actuator,
                                          xit->GetDouble("mechanicalReduction", 0.0, 1),
                                          xit->GetDouble("motorTorqueConstant", 0.0, 1),
                                          xit->GetDouble("pulsesPerRevolution", 0.0, 1)));

    }
    else
    {
      // TODO: report: Unknown transmission type
      continue;
    }
  }
#endif
}

void GazeboActuators::InitChild()
{
#if 0
  // Registers the actuators with MechanismControl
  assert(actuator_names_.size() == hw_.actuators_.size());
  for (unsigned int i = 0; i < actuator_names_.size(); ++i)
    mc_.registerActuator(actuator_names_[i], i);
#endif

  // TODO: mc_.init();

  hw_.current_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboActuators::UpdateChild()
{
  //--------------------------------------------------
  //  run the reattime mc update
  //--------------------------------------------------
  UpdateMC();

#if 0
  //--------------------------------------------------
  //
  //  from mc, get actuator data
  //  get joint data from actuator data
  //  put joint data into gazebo joints
  //
  //--------------------------------------------------
  //assert(gazebo_joints_.size() == mech_joints_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < gazebo_joints_.size(); ++i)
  {
    assert(gazebo_joints_[i]->GetType() == Joint::HINGE);
    HingeJoint *hj = (HingeJoint*)gazebo_joints_[i];
    mech_joints_[i]->position_ = hj->GetAngle();
    mech_joints_[i]->velocity_ = hj->GetAngleRate();
    mech_joints_[i]->applied_effort_ = mech_joints_[i]->commanded_effort_;
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  for (unsigned int i = 0; i < transmissions_.size(); ++i)
  {
    std::cout << " i " << i << ((mechanism::SimpleTransmission*)transmissions_[i])->mechanical_reduction_;
    transmissions_[i]->propagatePositionBackwards();
  }

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  mc_.update();

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  for (unsigned int i = 0; i < transmissions_.size(); ++i)
    transmissions_[i]->propagateEffortBackwards();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < gazebo_joints_.size(); ++i)
  {
    assert(gazebo_joints_[i]->GetType() == Joint::HINGE);
    HingeJoint *hj = (HingeJoint*)gazebo_joints_[i];
    hj->SetTorque(mech_joints_[i]->commanded_effort_);
  }
#endif
}

void GazeboActuators::FiniChild()
{

}

void GazeboActuators::LoadMC(XMLConfigNode *node)
{
  //-------------------------------------------------------------------------------------------
  //
  // GET INFORMATION FROM PR2.XML FROM PARAM SERVER
  // AND PUT IT INTO ROBOT_
  //
  // create a set of mech_joint_ for the robot and
  //
  // create a set of reverse_mech_joint_ for the reverse transmission results
  //
  //-------------------------------------------------------------------------------------------

  // get all links in pr2.xml
  std::vector<robot_desc::URDF::Link*> links;
  pr2Description.getLinks(links);
  std::cout << " pr2.xml link size(): " << links.size() << std::endl;

  // create a robot for forward transmission
  // create joints for mech_joint_ cycle through all links in pr2.xml
  mech_robot_ = new mechanism::Robot((char*)NULL);
  for (std::vector<robot_desc::URDF::Link*>::iterator lit = links.begin(); lit != links.end(); lit++)
  {
    std::cout << " link name: " << (*lit)->name << std::endl;
    if ((*lit)->isSet["joint"])
    {
      // FIXME: assume there's a joint to every link, this is not true if there are floating joints
      std::cout << " link joint name: " << (*lit)->joint->name << std::endl;
      mechanism::Joint* joint;
      joint = new mechanism::Joint();

      // FIXME: bug: copy name to a variable
      //char* robot_joint_name;
      //robot_joint_name        = new char((*lit)->joint->name.size());
      //std::cout << "size " << (*lit)->joint->name.size() << std::endl;
      //memcpy(robot_joint_name,(*lit)->joint->name.c_str(),(*lit)->joint->name.size());
      //joint->name_             = robot_joint_name; // does this save the string correctly?

      switch ((*lit)->joint->type)     // not used, it's an int
      {
        case robot_desc::URDF::Link::Joint::UNKNOWN:
          joint->type_         = mechanism::JOINT_NONE;
          break;
        case robot_desc::URDF::Link::Joint::FIXED:
          joint->type_         = mechanism::JOINT_FIXED;
          break;
        case robot_desc::URDF::Link::Joint::REVOLUTE:
          if ((*lit)->isSet["limit"])
            joint->type_       = mechanism::JOINT_ROTARY;
          else
            joint->type_       = mechanism::JOINT_CONTINUOUS;
          break;
        case robot_desc::URDF::Link::Joint::PRISMATIC:
          joint->type_         = mechanism::JOINT_PRISMATIC;
          break;
        case robot_desc::URDF::Link::Joint::PLANAR:
          joint->type_         = mechanism::JOINT_NONE;
          break;
        case robot_desc::URDF::Link::Joint::FLOATING:
          joint->type_         = mechanism::JOINT_NONE;
          break;
      }
      joint->initialized_      = true;  // from transmission
      joint->position_         = 0;     // from transmission
      joint->velocity_         = 0;     // from transmission
      joint->applied_effort_   = 0;     // from transmission
      joint->commanded_effort_ = 0;     // to transmission
      joint->joint_limit_min_  = 0;
      joint->joint_limit_max_  = 0;
      joint->effort_limit_     = (*lit)->joint->effortLimit;
      joint->velocity_limit_   = (*lit)->joint->velocityLimit;
      mech_robot_->joints_.push_back(joint);
      mech_robot_->joints_lookup_.insert(make_pair((*lit)->joint->name,(mech_robot_->joints_.size())-1));
    }
  }
  mech_robot_->hw_ = &hw_;



  // create a fake robot for reverse transmission in gazebo
  // create joints for reverse_mech_joint_, cycle through all links in pr2.xml
  reverse_mech_robot_ = new mechanism::Robot((char*)NULL);
  for (std::vector<robot_desc::URDF::Link*>::iterator lit = links.begin(); lit != links.end(); lit++)
  {
    std::cout << " link name: " << (*lit)->name << std::endl;
    if ((*lit)->isSet["joint"])
    {
      // FIXME: assume there's a joint to every link, this is not true if there are floating joints
      std::cout << " link joint name: " << (*lit)->joint->name << std::endl;
      mechanism::Joint* joint;
      joint = new mechanism::Joint();

      // FIXME: bug: copy name to a variable
      //char* robot_joint_name;
      //robot_joint_name        = new char((*lit)->joint->name.size());
      //std::cout << "size " << (*lit)->joint->name.size() << std::endl;
      //memcpy(robot_joint_name,(*lit)->joint->name.c_str(),(*lit)->joint->name.size());
      //joint->name_             = robot_joint_name; // does this save the string correctly?

      switch ((*lit)->joint->type)     // not used, it's an int
      {
        case robot_desc::URDF::Link::Joint::UNKNOWN:
          joint->type_         = mechanism::JOINT_NONE;
          break;
        case robot_desc::URDF::Link::Joint::FIXED:
          joint->type_         = mechanism::JOINT_FIXED;
          break;
        case robot_desc::URDF::Link::Joint::REVOLUTE:
          if ((*lit)->isSet["limit"])
            joint->type_       = mechanism::JOINT_ROTARY;
          else
            joint->type_       = mechanism::JOINT_CONTINUOUS;
          break;
        case robot_desc::URDF::Link::Joint::PRISMATIC:
          joint->type_         = mechanism::JOINT_PRISMATIC;
          break;
        case robot_desc::URDF::Link::Joint::PLANAR:
          joint->type_         = mechanism::JOINT_NONE;
          break;
        case robot_desc::URDF::Link::Joint::FLOATING:
          joint->type_         = mechanism::JOINT_NONE;
          break;
      }
      joint->initialized_      = true;  // from transmission
      joint->position_         = 0;     // from transmission
      joint->velocity_         = 0;     // from transmission
      joint->applied_effort_   = 0;     // from transmission
      joint->commanded_effort_ = 0;     // to transmission
      joint->joint_limit_min_  = 0;
      joint->joint_limit_max_  = 0;
      joint->effort_limit_     = (*lit)->joint->effortLimit;
      joint->velocity_limit_   = (*lit)->joint->velocityLimit;
      reverse_mech_robot_->joints_.push_back(joint);
      reverse_mech_robot_->joints_lookup_.insert(make_pair((*lit)->joint->name,(reverse_mech_robot_->joints_.size())-1));
    }
  }
  reverse_mech_robot_->hw_ = &hw_;



  // loop through copied controller, transmission, actuator data in gazebo pr2 model file
  for (XMLConfigNode *xit = node->GetChild("robot"); xit; xit=xit->GetNext("robot"))
  {
    //-----------------------------------------------------------------------------------------
    //
    // CONTROLLER XML
    //
    //-----------------------------------------------------------------------------------------
    std::cout << " LoadChild gazebo controller: " <<  xit->GetString("name","",0) << std::endl;

    // one layer below <robot name="pr2">
    // Reads the controllers information from the config.
    for (XMLConfigNode *cit = xit->GetChild("controller"); cit; cit = cit->GetNext("controller"))
    {
      Robot_controller_ controller;

      controller.name = cit->GetString("name", "", 1);
      controller.type = cit->GetString("type", "joint_controller", 1);

      XMLConfigNode *jit = cit->GetChild("joint");

      controller.joint_name = jit->GetString("name", "", 1);
      controller.joint_type = jit->GetString("type", "revolute", 0);

      // get a pointer to mech_robot_->joints_!
      mechanism::Robot::IndexMap::iterator mjit = mech_robot_->joints_lookup_.find(controller.joint_name);
      if (mjit == mech_robot_->joints_lookup_.end())
      {
        // TODO: report: Could not find the joint named xxxx
        std::cout << " join name " << controller.joint_name
                  << " not found, probably an abstract joint, like a gripper joint. " << std::endl;
        // FIXME: need to have a mechanism joint for controller to control!
        //        we can look at the finger joints below, and use one of them, or
        //        create a new joint: mech_robot_->joints_.insert( new_abstract_joint  );
        //continue; // skip, do not add controller


        //
        // artifically insert a gripper joint into mech_robot_->joints_
        //
        mechanism::Joint* joint;
        joint = new mechanism::Joint();

        joint->type_             = mechanism::JOINT_ROTARY;
        joint->initialized_      = true;  // from transmission
        joint->position_         = 0;     // from transmission
        joint->velocity_         = 0;     // from transmission
        joint->applied_effort_   = 0;     // from transmission
        joint->commanded_effort_ = 0;     // to transmission
        joint->joint_limit_min_  = 0;
        joint->joint_limit_max_  = 0;
        joint->effort_limit_     = (jit->GetChild("gripper_defaults"))->GetDouble("effortLimit",0,0);
        joint->velocity_limit_   = (jit->GetChild("gripper_defaults"))->GetDouble("velocityLimit",0,0);
        mech_robot_->joints_.push_back(joint);
        mech_robot_->joints_lookup_.insert(make_pair(controller.joint_name,(mech_robot_->joints_.size())-1));
        controller.mech_joint_ = mech_robot_->joints_.back();  // return joint we just added
      }
      else
      {
        controller.mech_joint_ = mech_robot_->joints_.at(mjit->second);  // we want to control this link
      }

      // get a pointer to reverse_mech_robot_->joints_!
      mechanism::Robot::IndexMap::iterator rmjit = reverse_mech_robot_->joints_lookup_.find(controller.joint_name);
      if (rmjit == reverse_mech_robot_->joints_lookup_.end())
      {
        // TODO: report: Could not find the joint named xxxx
        std::cout << " join name " << controller.joint_name
                  << " not found, probably an abstract joint, like a gripper joint. " << std::endl;
        // FIXME: need to have a mechanism joint for controller to control!
        //        we can look at the finger joints below, and use one of them, or
        //        create a new joint: reverse_mech_robot_->joints_.insert( new_abstract_joint  );
        //continue; // skip, do not add controller


        //
        // artifically insert a gripper joint into reverse_mech_robot_->joints_
        //
        mechanism::Joint* joint;
        joint = new mechanism::Joint();

        joint->type_             = mechanism::JOINT_ROTARY;
        joint->initialized_      = true;  // from transmission
        joint->position_         = 0;     // from transmission
        joint->velocity_         = 0;     // from transmission
        joint->applied_effort_   = 0;     // from transmission
        joint->commanded_effort_ = 0;     // to transmission
        joint->joint_limit_min_  = 0;
        joint->joint_limit_max_  = 0;
        joint->effort_limit_     = (jit->GetChild("gripper_defaults"))->GetDouble("effortLimit",0,0);
        joint->velocity_limit_   = (jit->GetChild("gripper_defaults"))->GetDouble("velocityLimit",0,0);
        reverse_mech_robot_->joints_.push_back(joint);
        reverse_mech_robot_->joints_lookup_.insert(make_pair(controller.joint_name,(reverse_mech_robot_->joints_.size())-1));
        controller.reverse_mech_joint_ = reverse_mech_robot_->joints_.back();  // return joint we just added
      }
      else
      {
        controller.reverse_mech_joint_ = reverse_mech_robot_->joints_.at(rmjit->second);  // we want to control this link
      }


      // setup pid controller
      XMLConfigNode *pit = jit->GetChild("pid_defaults");
      controller.control_mode = pit->GetString("controlMode", "PD_CONTROL", 0);
      controller.p_gain = pit->GetDouble("p", 1, 0);
      controller.i_gain = pit->GetDouble("i", 0, 0);
      controller.d_gain = pit->GetDouble("d", 0, 0);
      controller.windup = pit->GetDouble("iClamp", 0, 0);
      controller.init_time = Simulator::Instance()->GetSimTime();
      // initialize controller
      TiXmlElement junk("");
      if (controller.control_mode == "PD_CONTROL")
      {
        controller.pcontroller.initXml(mech_robot_,&junk); // just pass Robot pointer to controller.  controller uses hw_ to get time
        controller.pcontroller.init(controller.p_gain,controller.i_gain,controller.d_gain,controller.windup,controller.init_time,controller.mech_joint_);
      }
      else if (controller.control_mode == "VELOCITY_CONTROL")
      {
        controller.vcontroller.initXml(mech_robot_,&junk); // just pass Robot pointer to controller.  controller uses hw_ to get time
        controller.vcontroller.init(controller.p_gain,controller.i_gain,controller.d_gain,controller.windup,controller.init_time,controller.mech_joint_);
      }

      XMLConfigNode *dit = jit->GetChild("data");
      std::string data_name = dit->GetString("name","",1);
      std::string data_type = dit->GetString("type","",1);
      if (data_type == "gazebo") // check to see if it's for gazebo
      {
        for (XMLConfigNode *eit=dit->GetChild("elem"); eit ; eit=eit->GetNext("elem"))
        {
          controller.saturation_torque          = eit->GetDouble("saturationTorque",0,0);
          controller.explicitDampingCoefficient = eit->GetDouble("explicitDampingCoefficient",0,0);
          controller.gazebo_joint_type          = eit->GetString("type", "hinge_joint", 0);
          // special gazebo joint type
          if (controller.gazebo_joint_type == "gripper")
          {
              std::string f_l_joint     = eit->GetString("left_proximal","",1);
              std::string f_r_joint     = eit->GetString("right_proximal","",1);
              std::string f_tip_l_joint = eit->GetString("left_distal","",1);
              std::string f_tip_r_joint = eit->GetString("right_distal","",1);
              gazebo::HingeJoint* gj_f_l     = (gazebo::HingeJoint*)parent_model_->GetJoint(f_l_joint)    ;
              gazebo::HingeJoint* gj_f_r     = (gazebo::HingeJoint*)parent_model_->GetJoint(f_r_joint)    ;
              gazebo::HingeJoint* gj_f_tip_l = (gazebo::HingeJoint*)parent_model_->GetJoint(f_tip_l_joint);
              gazebo::HingeJoint* gj_f_tip_r = (gazebo::HingeJoint*)parent_model_->GetJoint(f_tip_r_joint);
              controller.gazebo_joints_.push_back(gj_f_l    );
              controller.gazebo_joints_.push_back(gj_f_r    );
              controller.gazebo_joints_.push_back(gj_f_tip_l);
              controller.gazebo_joints_.push_back(gj_f_tip_r);
              // initialize for torque control mode
              gj_f_l    ->SetParam(dParamVel , 0);
              gj_f_l    ->SetParam(dParamFMax, 0);
              gj_f_r    ->SetParam(dParamVel , 0);
              gj_f_r    ->SetParam(dParamFMax, 0);
              gj_f_tip_l->SetParam(dParamVel , 0);
              gj_f_tip_l->SetParam(dParamFMax, 0);
              gj_f_tip_r->SetParam(dParamVel , 0);
              gj_f_tip_r->SetParam(dParamFMax, 0);
          }
          else if (controller.gazebo_joint_type == "slider")
          {
              gazebo::SliderJoint* gjs     = (gazebo::SliderJoint*)parent_model_->GetJoint(controller.joint_name);
              controller.gazebo_joints_.push_back(gjs);
              gjs->SetParam(dParamVel , 0);
              gjs->SetParam(dParamFMax, 0);
          }
          else // defaults to hinge
          {
              gazebo::HingeJoint* gjh     = (gazebo::HingeJoint*)parent_model_->GetJoint(controller.joint_name);
              controller.gazebo_joints_.push_back(gjh);
              gjh->SetParam(dParamVel , 0);
              gjh->SetParam(dParamFMax, 0);
          }
        }
        std::cout << " controller name: "   << controller.name
                  << " controller type: "   << controller.type
                  << " joint name: "        << controller.joint_name
                  << " joint type: "        << controller.joint_type
                  << " gazebo joint size: " << controller.gazebo_joints_.size()
                  << " gazebo joint type: " << controller.gazebo_joint_type << std::endl;
      }
      robot_controllers_.push_back(controller);

    }

    //-----------------------------------------------------------------------------------------
    //
    // TRANSMISSION XML
    //
    //-----------------------------------------------------------------------------------------
    // Reads the transmission information from the config.
    for (XMLConfigNode *tit = xit->GetChild("transmission"); tit; tit = tit->GetNext("transmission"))
    {
      //==================================================================================================
      //for forward transmission (actuator -> real robot joint)
      Robot_transmission_ trn;
      trn.name           = tit->GetString("name", "", 1);

      trn.joint_name     =  tit->GetChild("joint")->GetString("name","",1);
      trn.actuator_name  =  tit->GetChild("actuator")->GetString("name","",1);

      trn.simple_transmission.mechanical_reduction_ = tit->GetDouble("mechanicalReduction",0,1);
      trn.simple_transmission.motor_torque_constant_= tit->GetDouble("motorTorqueConstant",0,1);
      trn.simple_transmission.pulses_per_revolution_= tit->GetDouble("pulsesPerRevolution",0,1);
      //trn.simple_transmission.actuator_ = ; // pointer to our actuator;
      //trn.simple_transmission.joint_    = ; // pointer to our robot joint;
      //trn.gazebo_joints_ = parent_model_->GetJoint(transmission.joint_name); // this is not necessary
      //assert(trn.gazebo_joints_ != NULL); // this is not necessary
      robot_transmissions_.push_back(trn);
      //==================================================================================================
      //for reverse transmission (actuator -> gazebo's fake robot joint copy
      Robot_transmission_ rtrn;
      rtrn.name           = tit->GetString("name", "", 1);

      rtrn.joint_name     =  tit->GetChild("joint")->GetString("name","",1);
      rtrn.actuator_name  =  tit->GetChild("actuator")->GetString("name","",1);

      rtrn.simple_transmission.mechanical_reduction_ = tit->GetDouble("mechanicalReduction",0,1);
      rtrn.simple_transmission.motor_torque_constant_= tit->GetDouble("motorTorqueConstant",0,1);
      rtrn.simple_transmission.pulses_per_revolution_= tit->GetDouble("pulsesPerRevolution",0,1);
      //rtrn.simple_transmission.actuator_ = ; // pointer to our actuator;
      //rtrn.simple_transmission.joint_    = ; // pointer to our robot joint;
      //rtrn.gazebo_joints_ = parent_model_->GetJoint(transmission.joint_name); // this is not necessary
      //assert(rtrn.gazebo_joints_ != NULL); // this is not necessary
      reverse_robot_transmissions_.push_back(rtrn);
      //==================================================================================================
      std::cout << " transmission name " << trn.name
                << " joint name    " << trn.joint_name
                << " actuator name " << trn.actuator_name
                << " mec red "       << trn.simple_transmission.mechanical_reduction_
                << " tor con "       << trn.simple_transmission.motor_torque_constant_
                << " pul rev "       << trn.simple_transmission.pulses_per_revolution_ << std::endl;
      //==================================================================================================
    }

    //-----------------------------------------------------------------------------------------
    //
    // ACTUATOR XML
    //
    //-----------------------------------------------------------------------------------------
    // Reads the actuator information from the config.
    for (XMLConfigNode *ait = xit->GetChild("actuator"); ait; ait = ait->GetNext("actuator"))
    {
      Robot_actuator_ actuator;
      // read from actuator_test.xml
      actuator.name             = ait->GetString("name", "", 1);
      actuator.motorboardID     = ait->GetString("motorboardID", "", 1);
      actuator.maxCurrent       = ait->GetDouble("maxCurrent", 0, 1);
      actuator.motor            = ait->GetString("motor", "", 1);
      actuator.ip               = ait->GetString("ip", "", 1);
      actuator.port             = ait->GetDouble("port", 0, 1);
      actuator.reduction        = ait->GetDouble("reduction", 0, 1);
      actuator.polymap          = ait->GetVector3("polymap",Vector3(1,0,0));

      // initialize the actuator object
      actuator.actuator.state_.encoder_count_ = 0;
      actuator.actuator.state_.timestamp_     = Simulator::Instance()->GetSimTime();
      actuator.actuator.state_.is_enabled_    = true;
      actuator.actuator.command_.enable_      = true;
      actuator.actuator.command_.current_     = 0;

      robot_actuators_.insert(make_pair(actuator.name,actuator));

      // formal structures
      hw_.actuators_.push_back(&actuator.actuator);
      //actuator_names_.push_back(actuator.name);

      std::cout << " actuator name " << actuator.name
                << " reduction " << actuator.reduction
                << " polymap " << actuator.polymap << std::endl;



    }


  }

  //==================================================================================================
  // fetch actuator and joint pair for forward transmission
  // loop through all transmissions
  for (std::vector<Robot_transmission_>::iterator rti = robot_transmissions_.begin(); rti != robot_transmissions_.end(); rti++)
  {
    // use actuator name to find actuator
    std::map<std::string,Robot_actuator_>::iterator mrai = robot_actuators_.find((*rti).actuator_name);
    assert (mrai != robot_actuators_.end()); // actuator must exist
    (*rti).simple_transmission.actuator_ = &((mrai->second).actuator);  // link actuator to transmission
    // use joint name to find mech_joint_ in controller, get a pointer to mech_robot_->joints_!
    mechanism::Robot::IndexMap::iterator mjit = mech_robot_->joints_lookup_.find((*rti).joint_name);
    assert (mjit != mech_robot_->joints_lookup_.end()); // joint must exist
    (*rti).simple_transmission.joint_ = mech_robot_->joints_.at(mjit->second);  // link joint to transmission
  }
  //==================================================================================================
  for (std::vector<Robot_transmission_>::iterator rrti = reverse_robot_transmissions_.begin(); rrti != reverse_robot_transmissions_.end(); rrti++)
  {
    // use actuator name to find actuator
    std::map<std::string,Robot_actuator_>::iterator mrai = robot_actuators_.find((*rrti).actuator_name);
    assert (mrai != robot_actuators_.end()); // actuator must exist
    (*rrti).simple_transmission.actuator_ = &((mrai->second).actuator);  // link actuator to transmission
    // use joint name to find reverse_mech_joint_ in controller, get a pointer to reverse_mech_robot_->joints_!
    mechanism::Robot::IndexMap::iterator rmjit = reverse_mech_robot_->joints_lookup_.find((*rrti).joint_name);
    assert (rmjit != reverse_mech_robot_->joints_lookup_.end()); // joint must exist
    (*rrti).simple_transmission.joint_ = reverse_mech_robot_->joints_.at(rmjit->second);  // link joint to transmission
  }
  //==================================================================================================


  //   interface = "/data/johnhsu/pr2/robot_descriptions/wg_robot_description/pr2/transmission_test.xml";
  //   xml_file  = "/data/johnhsu/pr2/robot_descriptions/wg_robot_description/pr2/pr2_test.xml";
  //  
  //   // load the robot xml
  //   TiXmlDocument xml(xml_file);
  //   xml.LoadFile();
  //   root = xml.FirstChildElement("robot");
  //  
  //   // setup hardware for the mechanism control
  //   // gazebo hardware is here, so all we need is some hardware interface!
  //   // setup hardware interface, transmissions between robot model and the hardware
  //  
  //   // TODO: register actuators with mechanism control
  //   mc_.init(root);
  //  
  //   // TODO: register some controller types
  //   mc_.registerControllerType("JointController", controller::JointController::create);
  //  
  //   // TODO: spawn some controllers
  //   mc_.spawnController("JointController", /*xml handle */NULL);
}

void GazeboActuators::UpdateMC()
{
  // pass time to robot
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  //---------------------------------------------------------------------
  // Real time update calls to mechanism control
  // this is what the hard real time loop does,
  // minus the tick() call to etherCAT
  //---------------------------------------------------------------------
  //
  // step through all controllers in the Robot_controller

  // update joint status from hardware
  for (std::vector<Robot_controller_>::iterator rci = robot_controllers_.begin(); rci != robot_controllers_.end() ; rci++)
  {
    if ((*rci).gazebo_joint_type == "gripper")
    {
      gazebo::HingeJoint* gj_f_l     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[0];
      gazebo::HingeJoint* gj_f_r     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[1];
      gazebo::HingeJoint* gj_f_tip_l = (gazebo::HingeJoint*) (*rci).gazebo_joints_[2];
      gazebo::HingeJoint* gj_f_tip_r = (gazebo::HingeJoint*) (*rci).gazebo_joints_[3];
      (*rci).mech_joint_->position_       = gj_f_l->GetAngle();
      (*rci).mech_joint_->velocity_       = gj_f_l->GetAngleRate();
      (*rci).mech_joint_->applied_effort_ = (*rci).mech_joint_->commanded_effort_;

    }
    else if ((*rci).gazebo_joint_type == "slider")
    {
      gazebo::SliderJoint* gjs  = (SliderJoint*)(*rci).gazebo_joints_[0];
      (*rci).mech_joint_->position_       = gjs->GetPosition();
      (*rci).mech_joint_->velocity_       = gjs->GetPositionRate();
      (*rci).mech_joint_->applied_effort_ = (*rci).mech_joint_->commanded_effort_;

    }
    else // defaults to hinge
    {
      gazebo::HingeJoint* gjh  = (HingeJoint*)(*rci).gazebo_joints_[0];
      (*rci).mech_joint_->position_       = gjh->GetAngle();
      (*rci).mech_joint_->velocity_       = gjh->GetAngleRate();
      (*rci).mech_joint_->applied_effort_ = (*rci).mech_joint_->commanded_effort_;

    }
  }

  // update each controller, this updates the joint that the controller was initialized with
  for (std::vector<Robot_controller_>::iterator rci = robot_controllers_.begin(); rci != robot_controllers_.end() ; rci++)
  {
    try
    {
      if ((*rci).control_mode == "PD_CONTROL")
      {
        (*rci).pcontroller.update();
      }
      else if ((*rci).control_mode == "VELOCITY_CONTROL")
      {
        (*rci).vcontroller.update();
      }

    }
    catch (char const* error)
    {
      std::cout << " controller update error " << error << std::endl;
    }
  }

  // update actuators from robot joints via forward transmission propagation
  for (std::vector<Robot_transmission_>::iterator rti = robot_transmissions_.begin(); rti != robot_transmissions_.end(); rti++)
  {
    // assign actuator states
    (*rti).simple_transmission.propagatePositionBackwards();
    // assign actuator commands
    (*rti).simple_transmission.propagateEffort();
  }

  //============================================================================================
  // below is when the actuator stuff goes to the hardware
  //============================================================================================

  // reverse transmission, get joint data from actuators
  for (std::vector<Robot_transmission_>::iterator rrti = reverse_robot_transmissions_.begin(); rrti != reverse_robot_transmissions_.end(); rrti++)
  {
    // assign joint states
    (*rrti).simple_transmission.propagatePosition();
    // assign joint effort
    (*rrti).simple_transmission.propagateEffortBackwards();
  }
    
  // udpate gazebo joint for this controller joint
  for (std::vector<Robot_controller_>::iterator rci = robot_controllers_.begin(); rci != robot_controllers_.end() ; rci++)
  {
    if ((*rci).gazebo_joint_type == "gripper")
    {
      // the 4 joints are hardcoded for our gripper
      gazebo::HingeJoint* gj_f_l     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[0];
      gazebo::HingeJoint* gj_f_r     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[1];
      gazebo::HingeJoint* gj_f_tip_l = (gazebo::HingeJoint*) (*rci).gazebo_joints_[2];
      gazebo::HingeJoint* gj_f_tip_r = (gazebo::HingeJoint*) (*rci).gazebo_joints_[3];
      double damp_force_f_l     = (*rci).explicitDampingCoefficient * gj_f_l    ->GetAngleRate();
      double damp_force_f_r     = (*rci).explicitDampingCoefficient * gj_f_r    ->GetAngleRate();
      double damp_force_f_tip_l = (*rci).explicitDampingCoefficient * gj_f_tip_l->GetAngleRate();
      double damp_force_f_tip_r = (*rci).explicitDampingCoefficient * gj_f_tip_r->GetAngleRate();
      gj_f_l->SetTorque(     (*rci).reverse_mech_joint_->commanded_effort_ - damp_force_f_l    );
      gj_f_r->SetTorque(    -(*rci).reverse_mech_joint_->commanded_effort_ - damp_force_f_r    );
      gj_f_tip_l->SetTorque(-(*rci).reverse_mech_joint_->commanded_effort_ - damp_force_f_tip_l);
      gj_f_tip_r->SetTorque( (*rci).reverse_mech_joint_->commanded_effort_ - damp_force_f_tip_r);
    }
    else if ((*rci).gazebo_joint_type == "slider")
    {
      gazebo::SliderJoint* gjs  = (SliderJoint*)(*rci).gazebo_joints_[0];
      gjs->SetSliderForce( (*rci).reverse_mech_joint_->commanded_effort_ );
    }
    else // defaults to hinge
    {
      gazebo::HingeJoint* gjh  = (HingeJoint*)(*rci).gazebo_joints_[0];
      double damp_force = (*rci).explicitDampingCoefficient * gjh->GetAngleRate();
      gjh->SetTorque( (*rci).reverse_mech_joint_->commanded_effort_ - damp_force);
    }


    if ((*rci).control_mode == "PD_CONTROL")
    {
      std::cout << " updating -- time is:"
                << Simulator::Instance()->GetSimTime()
                << "	updating controller:" << (*rci).name
                << "	command:" << (*rci).pcontroller.getCommand()
                << "	actual:" << (*rci).pcontroller.getActual()
                << std::endl;
    }
    else if ((*rci).control_mode == "VELOCITY_CONTROL")
    {
      std::cout << " updating -- time is:"
                << Simulator::Instance()->GetSimTime()
                << "	updating controller:" << (*rci).name
                << "	command:" << (*rci).vcontroller.getCommand()
                << "	actual:" << (*rci).vcontroller.getActual()
                << std::endl;
    }


  }


}

} // namespace gazebo
