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
    : Controller(parent) , hw_(0), mc_(&hw_), rmc_(&hw_) , mcn_(&mc_), rmcn_(&rmc_)
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
    tfs = new rosTFServer(*rosnode_); //, true, 1 * 1000000000ULL, 0ULL);

    // uses info from wg_robot_description_parser/send.xml
    std::string pr2Content;
    // get pr2.xml for Ioan's parser
    rosnode_->get_param("robotdesc/pr2",pr2Content);
    // parse the big pr2.xml string from ros
    pr2Description.loadString(pr2Content.c_str());


    // using tiny xml
    pr2Doc_ = new TiXmlDocument();
    pr2Doc_->SetUserData(NULL);
    pr2Doc_->Parse(pr2Content.c_str());

    AdvertiseSubscribeMessages();

  }

  GazeboActuators::~GazeboActuators()
  {
    //deleteElements(&gazebo_joints_);
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

  }

  void GazeboActuators::InitChild()
  {
    // TODO: mc_.init();

    hw_.current_time_ = Simulator::Instance()->GetSimTime();
  }

  void GazeboActuators::UpdateChild()
  {
    //--------------------------------------------------
    //  run the reattime mc update
    //--------------------------------------------------
    UpdateMC();

  }

  void GazeboActuators::FiniChild()
  {
    // TODO: will be replaced by global ros node eventually
    if (rosnode_ != NULL)
    {
      std::cout << "shutdown rosnode in test_actuators" << std::endl;
      //ros::fini();
      rosnode_->shutdown();
      //delete rosnode_;
    }
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

    // FIXME: mechanism control is not able to read joints from pr2.xml yet. for now, rely on Ioan's parser
    // get all links in pr2.xml
    std::vector<robot_desc::URDF::Link*> links;
    pr2Description.getLinks(links);
    std::cout << " pr2.xml link size: " << links.size() << std::endl;

    // create a robot for forward transmission
    // create joints for mech_joint_ cycle through all links in pr2.xml
    for (std::vector<robot_desc::URDF::Link*>::iterator lit = links.begin(); lit != links.end(); lit++)
    {
      std::cout << " link name: " << (*lit)->name;
      if ((*lit)->isSet["joint"])
      {
        // FIXME: assume there's a joint to every link, this is not true if there are floating joints
        mechanism::Joint* joint;
        joint = new mechanism::Joint();

        // assign name of joint
        joint->name_ = (*lit)->joint->name;
        //joint->name_ = new char(((*lit)->joint->name).size());
        //strcpy(joint->name_,(*lit)->joint->name.c_str());
        //std::cout << " link joint name: " << joint->name_ << std::endl;
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
        mc_.addJoint(joint);
      }
    }



    // create a fake robot for reverse transmission in gazebo
    // create joints for reverse_mech_joint_, cycle through all links in pr2.xml
    for (std::vector<robot_desc::URDF::Link*>::iterator lit = links.begin(); lit != links.end(); lit++)
    {
      std::cout << " link name: " << (*lit)->name;
      if ((*lit)->isSet["joint"])
      {
        // FIXME: assume there's a joint to every link, this is not true if there are floating joints
        mechanism::Joint* joint;
        joint = new mechanism::Joint();

        // assign name of joint
        joint->name_ = (*lit)->joint->name;
        //joint->name_ = new char(((*lit)->joint->name).size());
        //strcpy(joint->name_,(*lit)->joint->name.c_str());
        //std::cout << " link joint name: " << joint->name_ << std::endl;

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
        rmc_.addJoint(joint);
      }
    }



    //-----------------------------------------------------------------------------------------
    //
    // CONTROLLER XML
    //
    //-----------------------------------------------------------------------------------------
    for (XMLConfigNode *xit = node->GetChild("robot"); xit; xit=xit->GetNext("robot"))
    {
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

        // get a pointer to mc_->joints_!
        mechanism::Joint* j = mc_.model_.getJoint(controller.joint_name);

        if (j == NULL)
        {
          // TODO: report: Could not find the joint named xxxx
          std::cout << " join name " << controller.joint_name
                    << " not found, probably an abstract joint, like a gripper joint. " << std::endl;
          // FIXME: need to have a mechanism joint for controller to control!
          //        we can look at the finger joints below, and use one of them, or
          //        create a new joint: mc_.addJoint( new_abstract_joint  );
          //continue; // skip, do not add controller


          //
          // artifically insert a gripper joint into mc_.model_.joints_
          //
          mechanism::Joint* joint;
          joint = new mechanism::Joint();

          // assign name of joint
          joint->name_ = controller.joint_name;
          //joint->name_ = new char(controller.joint_name.size());
          //strcpy(joint->name_,controller.joint_name.c_str());
          //std::cout << " link joint name: " << joint->name_ << std::endl;


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
          mc_.addJoint(joint);
          controller.mech_joint_ = mc_.model_.joints_.back();  // return joint we just added
        }
        else
        {
          controller.mech_joint_ = j;  // we want to control this link
        }

        // get a pointer to rmc_->joints_!
        mechanism::Joint* rj = rmc_.model_.getJoint(controller.joint_name);
        if (rj == NULL)
        {
          // TODO: report: Could not find the joint named xxxx
          std::cout << " join name " << controller.joint_name
                    << " not found, probably an abstract joint, like a gripper joint. " << std::endl;
          // FIXME: need to have a mechanism joint for controller to control!
          //        we can look at the finger joints below, and use one of them, or
          //        create a new joint: rmc_.addJoint( new_abstract_joint  );
          //continue; // skip, do not add controller


          //
          // artifically insert a gripper joint into rmc_.model_.joints_
          //
          mechanism::Joint* joint;
          joint = new mechanism::Joint();

          // assign name of joint
          joint->name_ = controller.joint_name;
          //joint->name_ = new char(controller.joint_name.size());
          //strcpy(joint->name_,controller.joint_name.c_str());
          //std::cout << " link joint name: " << joint->name_ << std::endl;

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
          rmc_.addJoint(joint);

          controller.reverse_mech_joint_ = rmc_.model_.joints_.back();  // return joint we just added
        }
        else
        {
          controller.reverse_mech_joint_ = rj;  // we want to control this link
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

        mc_.spawnController(controller.control_mode,
                            controller.name,
                            controller.p_gain,controller.i_gain,controller.d_gain,controller.windup,
                            controller.init_time,
                            controller.mech_joint_);

        rmc_.spawnController(controller.control_mode,
                             controller.name,
                             controller.p_gain,controller.i_gain,controller.d_gain,controller.windup,
                             controller.init_time,
                             controller.reverse_mech_joint_);

        // setup gazebo joints
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
    }

    for (XMLConfigNode *xit = node->GetChild("robot"); xit; xit=xit->GetNext("robot"))
    {
      //-----------------------------------------------------------------------------------------
      //
      // ACTUATOR XML
      //
      //-----------------------------------------------------------------------------------------
      // Reads the actuator information from the config.
      for (XMLConfigNode *ait = xit->GetChild("actuator"); ait; ait = ait->GetNext("actuator"))
      {
        // read from actuator_test.xml
        std::string actuator_name          = ait->GetString("name", "", 1);
        // std::string motorboardID     = ait->GetString("motorboardID", "", 1);
        // double      maxCurrent       = ait->GetDouble("maxCurrent", 0, 1);
        // std::string motor            = ait->GetString("motor", "", 1);
        // std::string ip               = ait->GetString("ip", "", 1);
        // double      port             = ait->GetDouble("port", 0, 1);
        // double      reduction        = ait->GetDouble("reduction", 0, 1);
        // Vector3     polymap          = ait->GetVector3("polymap",Vector3(1,0,0));

        Actuator* actuator = new Actuator();
        // initialize the actuator object
        actuator->state_.encoder_count_ = 0;
        actuator->state_.timestamp_     = Simulator::Instance()->GetSimTime();
        actuator->state_.is_enabled_    = true;
        actuator->command_.enable_      = true;
        actuator->command_.current_     = 0;

        // formal structures
        // forward and revere mc_ share same hardware actuators
        hw_.actuators_.push_back(actuator);
        mc_.registerActuator(actuator_name,hw_.actuators_.size()-1);
        rmc_.registerActuator(actuator_name,hw_.actuators_.size()-1);

        std::cout << " adding actuator name to hw_ " << actuator_name << " " << hw_.actuators_.size() << std::endl;
      }
    }


    for (XMLConfigNode *xit = node->GetChild("robot"); xit; xit=xit->GetNext("robot"))
    {
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
        // FIXME: fix parsing in transmission so this is not needed
        mechanism::SimpleTransmission st; // = new mechanism::SimpleTransmission();
        transmissions_.push_back(st);
        //==================================================================================================
        transmissions_.back().name_           = tit->GetString("name", "", 1);
        transmissions_.back().joint_name_     =  tit->GetChild("joint")->GetString("name","",1);
        transmissions_.back().actuator_name_  =  tit->GetChild("actuator")->GetString("name","",1);
        transmissions_.back().mechanical_reduction_ = tit->GetDouble("mechanicalReduction",0,1);
        transmissions_.back().motor_torque_constant_= tit->GetDouble("motorTorqueConstant",0,1);
        transmissions_.back().pulses_per_revolution_= tit->GetDouble("pulsesPerRevolution",0,1);
        mc_.addSimpleTransmission(&transmissions_.back());
        //==================================================================================================
        //for reverse transmission (actuator -> gazebo's fake robot joint copy
        rmc_.addSimpleTransmission(&transmissions_.back());
        //==================================================================================================
        std::cout << " adding transmission name " << transmissions_.back().name_
                  << " joint name    " << transmissions_.back().joint_name_
                  << " actuator name " << transmissions_.back().actuator_name_
                  << " mec red "       << transmissions_.back().mechanical_reduction_
                  << " tor con "       << transmissions_.back().motor_torque_constant_
                  << " pul rev "       << transmissions_.back().pulses_per_revolution_ << std::endl;
      }
    }

  }


  //-------------------------------------------------------------------------------------------//
  //                                                                                           //
  //                                                                                           //
  //                                                                                           //
  //  MAIN UPDATE LOOP FOR MC AND ROS PUBLISH                                                  //
  //                                                                                           //
  //                                                                                           //
  //                                                                                           //
  //-------------------------------------------------------------------------------------------//
  void GazeboActuators::UpdateMC()
  {
    // pass time to robot
    hw_.current_time_ = Simulator::Instance()->GetSimTime();


    this->lock.lock();
    /***************************************************************/
    /*                                                             */
    /*  publish time to ros                                        */
    /*                                                             */
    /***************************************************************/
    timeMsg.rostime.sec  = (unsigned long)floor(hw_.current_time_);
    timeMsg.rostime.nsec = (unsigned long)floor(  1e9 * (  hw_.current_time_ - timeMsg.rostime.sec) );
    rosnode_->publish("time",timeMsg);
    /***************************************************************/
    /*                                                             */
    /*  odometry                                                   */
    /*                                                             */
    /***************************************************************/
    // Get latest odometry data
    // Get velocities
    double vx,vy,vw;
    //this->PR2Copy->GetBaseCartesianSpeedActual(&vx,&vy,&vw);
    // Translate into ROS message format and publish
    this->odomMsg.vel.x  = vx;
    this->odomMsg.vel.y  = vy;
    this->odomMsg.vel.th = vw;

    // Get position
    double x,y,z,roll,pitch,yaw;
    //this->PR2Copy->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&yaw);
    this->odomMsg.pos.x  = x;
    this->odomMsg.pos.y  = y;
    this->odomMsg.pos.th = yaw;

    // TODO: get the frame ID from somewhere
    this->odomMsg.header.frame_id = tfs->nameClient.lookup("FRAMEID_ODOM");

    this->odomMsg.header.stamp.sec = (unsigned long)floor(hw_.current_time_);
    this->odomMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  hw_.current_time_ - this->odomMsg.header.stamp.sec) );

    // This publish call resets odomMsg.header.stamp.sec and 
    // odomMsg.header.stamp.nsec to zero.  Thus, it must be called *after*
    // those values are reused in the sendInverseEuler() call above.
    rosnode_->publish("odom",this->odomMsg);

    /***************************************************************/
    /*                                                             */
    /*   object position                                           */
    /*                                                             */
    /***************************************************************/
    //this->PR2Copy->GetObjectPositionActual(&x,&y,&z,&roll,&pitch,&yaw);
    this->objectPosMsg.x  = x;
    this->objectPosMsg.y  = y;
    this->objectPosMsg.z  = z;
    rosnode_->publish("object_position", this->objectPosMsg);


    this->lock.unlock();

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
      // push gazebo joint to reverse joints
      if ((*rci).gazebo_joint_type == "gripper")
      {
        gazebo::HingeJoint* gj_f_l     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[0];
        gazebo::HingeJoint* gj_f_r     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[1];
        gazebo::HingeJoint* gj_f_tip_l = (gazebo::HingeJoint*) (*rci).gazebo_joints_[2];
        gazebo::HingeJoint* gj_f_tip_r = (gazebo::HingeJoint*) (*rci).gazebo_joints_[3];

        // controller::Controller* gc = mc_.getControllerByName( (*rci).name );
        double f_l_error     = ( gj_f_l     -> GetAngle() );
        // double f_r_error     = (-gj_f_r     -> GetAngle() );
        // double f_tip_l_error = (-gj_f_tip_l -> GetAngle() );
        // double f_tip_r_error = ( gj_f_tip_l -> GetAngle() );
        // std::cout << "getting controller error from mc : " << (*rci).name
        //           << " e1 : " << f_l_error     
        //           << " e2 : " << f_r_error     
        //           << " e3 : " << f_tip_l_error 
        //           << " e4 : " << f_tip_r_error 
        //           << " cmd "  << dynamic_cast<controller::JointPositionController*>(gc)->getCommand() << std::endl;
        //(*rci).reverse_mech_joint_->position_       = 0.25*(f_l_error + f_r_error + f_tip_l_error + f_tip_r_error); // average all positions

        (*rci).reverse_mech_joint_->position_       = f_l_error; // use one joint as reference
        (*rci).reverse_mech_joint_->velocity_       = gj_f_l->GetAngleRate();
        (*rci).reverse_mech_joint_->applied_effort_ = (*rci).reverse_mech_joint_->commanded_effort_;

      }
      else if ((*rci).gazebo_joint_type == "slider")
      {
        gazebo::SliderJoint* gjs  = (SliderJoint*)(*rci).gazebo_joints_[0];
        (*rci).reverse_mech_joint_->position_       = gjs->GetPosition();
        (*rci).reverse_mech_joint_->velocity_       = gjs->GetPositionRate();
        (*rci).reverse_mech_joint_->applied_effort_ = (*rci).reverse_mech_joint_->commanded_effort_;

      }
      else // defaults to hinge
      {
        gazebo::HingeJoint* gjh  = (HingeJoint*)(*rci).gazebo_joints_[0];
        (*rci).reverse_mech_joint_->position_       = gjh->GetAngle();
        (*rci).reverse_mech_joint_->velocity_       = gjh->GetAngleRate();
        (*rci).reverse_mech_joint_->applied_effort_ = (*rci).reverse_mech_joint_->commanded_effort_;

      }
    }
    // push reverse_mech_joint_ stuff back toward actuators
    for (unsigned int i=0; i < rmc_.model_.transmissions_.size(); i++)
    {
      rmc_.model_.transmissions_[i]->propagatePositionBackwards();
      rmc_.model_.transmissions_[i]->propagateEffort();
      std::cout << " applying reverse transmisison : "
                <<  dynamic_cast<mechanism::SimpleTransmission*>(rmc_.model_.transmissions_[i])->name_
                << " " <<  std::endl;
    }


    // -------------------------------------------------------------------------------------------------
    // -                                                                                               -
    // -   test some controllers set points by hardcode for debug                                      -
    // -                                                                                               -
    // -------------------------------------------------------------------------------------------------
    // set through ros?
    // artifically set command
    controller::Controller* mcc = mc_.getControllerByName( "shoulder_pitch_left_controller" );
    dynamic_cast<controller::JointPositionController*>(mcc)->setCommand(0.2);

    // -------------------------------------------------------------------------------------------------
    // -                                                                                               -
    // -  update each controller, this updates the joint that the controller was initialized with      -
    // -                                                                                               -
    // -  update mc given the actuator states are filled from above                                    -
    // -                                                                                               -
    // -  update actuators from robot joints via forward transmission propagation                      -
    // -                                                                                               -
    // -------------------------------------------------------------------------------------------------
    mc_.update();


    //============================================================================================
    // below is when the actuator stuff goes to the hardware
    //============================================================================================

    // -------------------------------------------------------------------------------------------------
    // -                                                                                               -
    // -    reverse transmission, get joint data from actuators                                        -
    // -                                                                                               -
    // -------------------------------------------------------------------------------------------------
    // propagate actuator data back to reverse-joints
    for (unsigned int i=0; i < rmc_.model_.transmissions_.size(); i++)
    {
      // assign reverse joint states from actuator states
      rmc_.model_.transmissions_[i]->propagatePosition();
      // assign joint effort
      rmc_.model_.transmissions_[i]->propagateEffortBackwards();
    }

    // -------------------------------------------------------------------------------------------------
    // -                                                                                               -
    // -     udpate gazebo joint for this controller joint                                             -
    // -                                                                                               -
    // -------------------------------------------------------------------------------------------------
    for (std::vector<Robot_controller_>::iterator rci = robot_controllers_.begin(); rci != robot_controllers_.end() ; rci++)
    {
      if ((*rci).gazebo_joint_type == "gripper")
      {
        // the 4 joints are hardcoded for our gripper
        gazebo::HingeJoint* gj_f_l     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[0];
        gazebo::HingeJoint* gj_f_r     = (gazebo::HingeJoint*) (*rci).gazebo_joints_[1];
        gazebo::HingeJoint* gj_f_tip_l = (gazebo::HingeJoint*) (*rci).gazebo_joints_[2];
        gazebo::HingeJoint* gj_f_tip_r = (gazebo::HingeJoint*) (*rci).gazebo_joints_[3];

        controller::Controller* gc = mc_.getControllerByName( (*rci).name );

        double f_l_error     = gj_f_l     -> GetAngle() - dynamic_cast<controller::JointPositionController*>(gc)->getCommand();
        double f_r_error     = gj_f_r     -> GetAngle() + dynamic_cast<controller::JointPositionController*>(gc)->getCommand();
        double f_tip_l_error = gj_f_tip_l -> GetAngle() + dynamic_cast<controller::JointPositionController*>(gc)->getCommand();
        double f_tip_r_error = gj_f_tip_r -> GetAngle() - dynamic_cast<controller::JointPositionController*>(gc)->getCommand();
        const double correctionConstant = 0.01;

        std::cout << "getting controller from mc : " << (*rci).name
                  << " e1 : " << f_l_error     << " c1 : " << 0.0*correctionConstant*f_l_error 
                  << " e2 : " << f_r_error     << " c2 : " << correctionConstant*f_r_error     
                  << " e3 : " << f_tip_l_error << " c3 : " << correctionConstant*f_tip_l_error 
                  << " e4 : " << f_tip_r_error << " c4 : " << correctionConstant*f_tip_r_error 
                  << " cmd "  << dynamic_cast<controller::JointPositionController*>(gc)->getCommand() << std::endl;
        std::cout << " current " << gj_f_tip_r -> GetAngle() 
                  << " target "  <<  dynamic_cast<controller::JointPositionController*>(gc)->getCommand()
                  << " force  "  <<  correctionConstant*f_tip_r_error
                  << std::endl;
        double damp_force_f_l     = (*rci).explicitDampingCoefficient * gj_f_l    ->GetAngleRate();
        double damp_force_f_r     = (*rci).explicitDampingCoefficient * gj_f_r    ->GetAngleRate();
        double damp_force_f_tip_l = (*rci).explicitDampingCoefficient * gj_f_tip_l->GetAngleRate();
        double damp_force_f_tip_r = (*rci).explicitDampingCoefficient * gj_f_tip_r->GetAngleRate();

        gj_f_l    ->SetTorque( (*rci).reverse_mech_joint_->commanded_effort_ + correctionConstant*f_l_error     - damp_force_f_l    );
        gj_f_r    ->SetTorque(-(*rci).reverse_mech_joint_->commanded_effort_ - correctionConstant*f_r_error     - damp_force_f_r    );
        gj_f_tip_l->SetTorque(-(*rci).reverse_mech_joint_->commanded_effort_ - correctionConstant*f_tip_l_error - damp_force_f_tip_l);
        gj_f_tip_r->SetTorque( (*rci).reverse_mech_joint_->commanded_effort_ + correctionConstant*f_tip_r_error - damp_force_f_tip_r);

        // std::cout << " updating gripper ----------------------------- " << std::endl;
        // std::cout << " f_l " <<  (*rci).reverse_mech_joint_->commanded_effort_ << " " << damp_force_f_l    << std::endl;
        // std::cout << " f_r " << -(*rci).reverse_mech_joint_->commanded_effort_ << " " << damp_force_f_r    << std::endl;
        // std::cout << " ftl " << -(*rci).reverse_mech_joint_->commanded_effort_ << " " << damp_force_f_tip_l<< std::endl;
        // std::cout << " ftr " <<  (*rci).reverse_mech_joint_->commanded_effort_ << " " << damp_force_f_tip_r<< std::endl;

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


      std::cout << " updating -- time is:"
                << Simulator::Instance()->GetSimTime()
                << "	updating controller:" << (*rci).name
                  << std::endl;

    }


  }

  void
  GazeboActuators::LoadFrameTransformOffsets()
  {
    // get all links in pr2.xml
    robot_desc::URDF::Link* link;

    // get all the parameters needed for frame transforms
    link = pr2Description.getLink("base");
    base_center_offset_z = link->collision->xyz[2];
    link = pr2Description.getLink("torso");
    base_torso_offset_x  = link->xyz[0];
    base_torso_offset_y  = link->xyz[1];
    base_torso_offset_z  = link->xyz[2];
    link = pr2Description.getLink("shoulder_pan_left");
    sh_pan_left_torso_offset_x =  link->xyz[0];
    sh_pan_left_torso_offset_y =  link->xyz[1];
    sh_pan_left_torso_offset_z =  link->xyz[2];
    link = pr2Description.getLink("shoulder_pitch_left");
    shoulder_pitch_left_offset_x = link->xyz[0];
    shoulder_pitch_left_offset_y = link->xyz[1];
    shoulder_pitch_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("upperarm_roll_left");
    upperarm_roll_left_offset_x = link->xyz[0];
    upperarm_roll_left_offset_y = link->xyz[1];
    upperarm_roll_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("elbow_flex_left");
    elbow_flex_left_offset_x = link->xyz[0];
    elbow_flex_left_offset_y = link->xyz[1];
    elbow_flex_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_l_left");
    finger_l_left_offset_x = link->xyz[0];
    finger_l_left_offset_y = link->xyz[1];
    finger_l_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("forearm_roll_left");
    forearm_roll_left_offset_x = link->xyz[0];
    forearm_roll_left_offset_y = link->xyz[1];
    forearm_roll_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("wrist_flex_left");
    wrist_flex_left_offset_x = link->xyz[0];
    wrist_flex_left_offset_y = link->xyz[1];
    wrist_flex_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("gripper_roll_left");
    gripper_roll_left_offset_x = link->xyz[0];
    gripper_roll_left_offset_y = link->xyz[1];
    gripper_roll_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_r_left");
    finger_r_left_offset_x = link->xyz[0];
    finger_r_left_offset_y = link->xyz[1];
    finger_r_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_tip_l_left");
    finger_tip_l_left_offset_x = link->xyz[0];
    finger_tip_l_left_offset_y = link->xyz[1];
    finger_tip_l_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_tip_r_left");
    finger_tip_r_left_offset_x = link->xyz[0];
    finger_tip_r_left_offset_y = link->xyz[1];
    finger_tip_r_left_offset_z = link->xyz[2];


    link = pr2Description.getLink("shoulder_pan_right");
    shoulder_pan_right_offset_x = link->xyz[0];
    shoulder_pan_right_offset_y = link->xyz[1];
    shoulder_pan_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("shoulder_pitch_right");
    shoulder_pitch_right_offset_x = link->xyz[0];
    shoulder_pitch_right_offset_y = link->xyz[1];
    shoulder_pitch_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("upperarm_roll_right");
    upperarm_roll_right_offset_x = link->xyz[0];
    upperarm_roll_right_offset_y = link->xyz[1];
    upperarm_roll_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("elbow_flex_right");
    elbow_flex_right_offset_x = link->xyz[0];
    elbow_flex_right_offset_y = link->xyz[1];
    elbow_flex_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("forearm_roll_right");
    forearm_roll_right_offset_x = link->xyz[0];
    forearm_roll_right_offset_y = link->xyz[1];
    forearm_roll_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("wrist_flex_right");
    wrist_flex_right_offset_x = link->xyz[0];
    wrist_flex_right_offset_y = link->xyz[1];
    wrist_flex_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("gripper_roll_right");
    gripper_roll_right_offset_x = link->xyz[0];
    gripper_roll_right_offset_y = link->xyz[1];
    gripper_roll_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_l_right");
    finger_l_right_offset_x = link->xyz[0];
    finger_l_right_offset_y = link->xyz[1];
    finger_l_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_r_right");
    finger_r_right_offset_x = link->xyz[0];
    finger_r_right_offset_y = link->xyz[1];
    finger_r_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_tip_l_right");
    finger_tip_l_right_offset_x = link->xyz[0];
    finger_tip_l_right_offset_y = link->xyz[1];
    finger_tip_l_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("finger_tip_r_right");
    finger_tip_r_right_offset_x = link->xyz[0];
    finger_tip_r_right_offset_y = link->xyz[1];
    finger_tip_r_right_offset_z = link->xyz[2];

    link = pr2Description.getLink("forearm_camera_left");
    forearm_camera_left_offset_x = link->xyz[0];
    forearm_camera_left_offset_y = link->xyz[1];
    forearm_camera_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("forearm_camera_right");
    forearm_camera_right_offset_x = link->xyz[0];
    forearm_camera_right_offset_y = link->xyz[1];
    forearm_camera_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("wrist_camera_left");
    wrist_camera_left_offset_x = link->xyz[0];
    wrist_camera_left_offset_y = link->xyz[1];
    wrist_camera_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("wrist_camera_right");
    wrist_camera_right_offset_x = link->xyz[0];
    wrist_camera_right_offset_y = link->xyz[1];
    wrist_camera_right_offset_z = link->xyz[2];


    link = pr2Description.getLink("head_pan");
    head_pan_offset_x = link->xyz[0];
    head_pan_offset_y = link->xyz[1];
    head_pan_offset_z = link->xyz[2];
    link = pr2Description.getLink("head_tilt");
    head_tilt_offset_x = link->xyz[0];
    head_tilt_offset_y = link->xyz[1];
    head_tilt_offset_z = link->xyz[2];
    link = pr2Description.getLink("base_laser");
    base_laser_offset_x = link->xyz[0];
    base_laser_offset_y = link->xyz[1];
    base_laser_offset_z = link->xyz[2];
    link = pr2Description.getLink("tilt_laser");
    tilt_laser_offset_x = link->xyz[0];
    tilt_laser_offset_y = link->xyz[1];
    tilt_laser_offset_z = link->xyz[2],
    link = pr2Description.getLink("caster_front_left");
    caster_front_left_offset_x = link->xyz[0];
    caster_front_left_offset_y = link->xyz[1];
    caster_front_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_front_left_l");
    wheel_front_left_l_offset_x = link->xyz[0];
    wheel_front_left_l_offset_y = link->xyz[1];
    wheel_front_left_l_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_front_left_r");
    wheel_front_left_r_offset_x = link->xyz[0];
    wheel_front_left_r_offset_y = link->xyz[1];
    wheel_front_left_r_offset_z = link->xyz[2];
    link = pr2Description.getLink("caster_front_right");
    caster_front_right_offset_x = link->xyz[0];
    caster_front_right_offset_y = link->xyz[1];
    caster_front_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_front_right_l");
    wheel_front_right_l_offset_x = link->xyz[0];
    wheel_front_right_l_offset_y = link->xyz[1];
    wheel_front_right_l_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_front_right_r");
    wheel_front_right_r_offset_x = link->xyz[0];
    wheel_front_right_r_offset_y = link->xyz[1];
    wheel_front_right_r_offset_z = link->xyz[2];
    link = pr2Description.getLink("caster_rear_left");
    caster_rear_left_offset_x = link->xyz[0];
    caster_rear_left_offset_y = link->xyz[1];
    caster_rear_left_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_rear_left_l");
    wheel_rear_left_l_offset_x = link->xyz[0];
    wheel_rear_left_l_offset_y = link->xyz[1];
    wheel_rear_left_l_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_rear_left_r");
    wheel_rear_left_r_offset_x = link->xyz[0];
    wheel_rear_left_r_offset_y = link->xyz[1];
    wheel_rear_left_r_offset_z = link->xyz[2];
    link = pr2Description.getLink("caster_rear_right");
    caster_rear_right_offset_x = link->xyz[0];
    caster_rear_right_offset_y = link->xyz[1];
    caster_rear_right_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_rear_right_l");
    wheel_rear_right_l_offset_x = link->xyz[0];
    wheel_rear_right_l_offset_y = link->xyz[1];
    wheel_rear_right_l_offset_z = link->xyz[2];
    link = pr2Description.getLink("wheel_rear_right_r");
    wheel_rear_right_r_offset_x = link->xyz[0];
    wheel_rear_right_r_offset_y = link->xyz[1];
    wheel_rear_right_r_offset_z = link->xyz[2];


  }




  int
  GazeboActuators::AdvertiseSubscribeMessages()
  {
    rosnode_->advertise<std_msgs::RobotBase2DOdom>("odom");
    rosnode_->advertise<std_msgs::PR2Arm>("left_pr2arm_pos");
    rosnode_->advertise<std_msgs::PR2Arm>("right_pr2arm_pos");
    rosnode_->advertise<rostools::Time>("time");
    rosnode_->advertise<std_msgs::Empty>("transform");
    rosnode_->advertise<std_msgs::Point3DFloat32>("object_position");

    rosnode_->subscribe("cmd_vel", velMsg, &GazeboActuators::CmdBaseVelReceived);
    rosnode_->subscribe("cmd_leftarmconfig", leftarmMsg, &GazeboActuators::CmdLeftarmconfigReceived);
    rosnode_->subscribe("cmd_rightarmconfig", rightarmMsg, &GazeboActuators::CmdRightarmconfigReceived);
    rosnode_->subscribe("cmd_leftarm_cartesian", leftarmcartesianMsg, &GazeboActuators::CmdLeftarmcartesianReceived);
    rosnode_->subscribe("cmd_rightarm_cartesian", rightarmcartesianMsg, &GazeboActuators::CmdRightarmcartesianReceived);
    
    return(0);
  }

  void
  GazeboActuators::CmdLeftarmconfigReceived()
  {
    this->lock.lock();
    this->lock.unlock();
  }
  void
  GazeboActuators::CmdRightarmconfigReceived()
  {
    this->lock.lock();
    this->lock.unlock();
  }

  void
  GazeboActuators::CmdLeftarmcartesianReceived()
  {
    this->lock.lock();
    this->lock.unlock();
  }
  void
  GazeboActuators::CmdRightarmcartesianReceived()
  {
    this->lock.lock();
    this->lock.unlock();
  }


  void
  GazeboActuators::CmdBaseVelReceived()
  {
    this->lock.lock();
    this->lock.unlock();
  }


  void
  GazeboActuators::PublishFrameTransforms()
  {

    /***************************************************************/
    /*                                                             */
    /*  frame transforms                                           */
    /*                                                             */
    /*  TODO: should we send z, roll, pitch, yaw? seems to confuse */
    /*        localization                                         */
    /*                                                             */
    /***************************************************************/
    double x,y,z,roll,pitch,yaw;
    //this->PR2Copy->GetBasePositionActual(&x,&y,&z,&roll,&pitch,&yaw); // actual CoM of base

    tfs->sendInverseEuler("FRAMEID_ODOM",
                 "base",
                 x,
                 y,
                 z - base_center_offset_z, /* get infor from xml: half height of base box */
                 yaw,
                 pitch,
                 roll,
                 odomMsg.header.stamp);

    /***************************************************************/
    /*                                                             */
    /*  frame transforms                                           */
    /*                                                             */
    /*  x,y,z,yaw,pitch,roll                                       */
    /*                                                             */
    /***************************************************************/
    tfs->sendEuler("base",
                 "FRAMEID_ROBOT",
                 0,
                 0,
                 0, 
                 0,
                 0,
                 0,
                 odomMsg.header.stamp);

    //std::cout << "base y p r " << yaw << " " << pitch << " " << roll << std::endl;

    // base = center of the bottom of the base box
    // torso = midpoint of bottom of turrets

    tfs->sendEuler("torso",
                 "base",
                 base_torso_offset_x,
                 base_torso_offset_y,
                 base_torso_offset_z, /* FIXME: spine elevator not accounted for */
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // arm_l_turret = bottom of left turret
    tfs->sendEuler("shoulder_pan_left",
                 "torso",
                 sh_pan_left_torso_offset_x,
                 sh_pan_left_torso_offset_y,
                 sh_pan_left_torso_offset_z,
                 0.0, //larm.turretAngle,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    //std::cout << "left pan angle " << larm.turretAngle << std::endl;

    // arm_l_shoulder = center of left shoulder pitch bracket
    tfs->sendEuler("shoulder_pitch_left",
                 "shoulder_pan_left",
                 shoulder_pitch_left_offset_x,
                 shoulder_pitch_left_offset_y,
                 shoulder_pitch_left_offset_z,
                 0.0,
                 0.0, //larm.shoulderLiftAngle,
                 0.0,
                 odomMsg.header.stamp);

    // arm_l_upperarm = upper arm with roll DOF, at shoulder pitch center
    tfs->sendEuler("upperarm_roll_left",
                 "shoulder_pitch_left",
                 upperarm_roll_left_offset_x,
                 upperarm_roll_left_offset_y,
                 upperarm_roll_left_offset_z,
                 0.0,
                 0.0,
                 0.0, //larm.upperarmRollAngle,
                 odomMsg.header.stamp);

    //frameid_arm_l_elbow = elbow pitch bracket center of rotation
    tfs->sendEuler("elbow_flex_left",
                 "upperarm_roll_left",
                 elbow_flex_left_offset_x,
                 elbow_flex_left_offset_y,
                 elbow_flex_left_offset_z,
                 0.0,
                 0.0, //larm.elbowAngle,
                 0.0,
                 odomMsg.header.stamp);

    //frameid_arm_l_forearm = forearm roll DOR, at elbow pitch center
    tfs->sendEuler("forearm_roll_left",
                 "elbow_flex_left",
                 forearm_roll_left_offset_x,
                 forearm_roll_left_offset_y,
                 forearm_roll_left_offset_z,
                 0.0,
                 0.0,
                 0.0, //larm.forearmRollAngle,
                 odomMsg.header.stamp);

    // arm_l_wrist = wrist pitch DOF.
    tfs->sendEuler("wrist_flex_left",
                 "forearm_roll_left",
                 wrist_flex_left_offset_x,
                 wrist_flex_left_offset_y,
                 wrist_flex_left_offset_z,
                 0.0,
                 0.0, //larm.wristPitchAngle,
                 0.0,
                 odomMsg.header.stamp);

    // arm_l_hand = hand roll DOF, center at wrist pitch center
    tfs->sendEuler("gripper_roll_left",
                 "wrist_flex_left",
                 gripper_roll_left_offset_x,
                 gripper_roll_left_offset_y,
                 gripper_roll_left_offset_z,
                 0.0,
                 0.0,
                 0.0, //larm.wristRollAngle,
                 odomMsg.header.stamp);

    // proximal digit, left
    tfs->sendEuler("finger_l_left",
                 "gripper_roll_left",
                 finger_l_left_offset_x,
                 finger_l_left_offset_y,
                 finger_l_left_offset_z,
                 0.0,  //FIXME: get angle of finger...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // proximal digit, right
    tfs->sendEuler("finger_r_left",
                 "gripper_roll_left",
                 finger_r_left_offset_x,
                 finger_r_left_offset_y,
                 finger_r_left_offset_z,
                 0.0,  //FIXME: get angle of finger...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // distal digit, left
    tfs->sendEuler("finger_tip_l_left",
                 "finger_l_left",
                 finger_tip_l_left_offset_x,
                 finger_tip_l_left_offset_y,
                 finger_tip_l_left_offset_z,
                 0.0,  //FIXME: get angle of finger tip...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // distal digit, left
    tfs->sendEuler("finger_tip_r_left",
                 "finger_r_left",
                 finger_tip_r_left_offset_x,
                 finger_tip_r_left_offset_y,
                 finger_tip_r_left_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);




    // arm_r_turret = bottom of right turret
    tfs->sendEuler("shoulder_pan_right",
                 "torso",
                 shoulder_pan_right_offset_x,
                 shoulder_pan_right_offset_y,
                 shoulder_pan_right_offset_z,
                 0.0, //rarm.turretAngle,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    //std::cout << "right pan angle " << larm.turretAngle << std::endl;

    // arm_r_shoulder = center of right shoulder pitch bracket
    tfs->sendEuler("shoulder_pitch_right",
                 "shoulder_pan_right",
                 shoulder_pitch_right_offset_x,
                 shoulder_pitch_right_offset_y,
                 shoulder_pitch_right_offset_z,
                 0.0,
                 0.0, //rarm.shoulderLiftAngle,
                 0.0,
                 odomMsg.header.stamp);

    // arm_r_upperarm = upper arm with roll DOF, at shoulder pitch center
    tfs->sendEuler("upperarm_roll_right",
                 "shoulder_pitch_right",
                 upperarm_roll_right_offset_x,
                 upperarm_roll_right_offset_y,
                 upperarm_roll_right_offset_z,
                 0.0,
                 0.0,
                 0.0, //rarm.upperarmRollAngle,
                 odomMsg.header.stamp);

    //frameid_arm_r_elbow = elbow pitch bracket center of rotation
    tfs->sendEuler("elbow_flex_right",
                 "upperarm_roll_right",
                 elbow_flex_right_offset_x,
                 elbow_flex_right_offset_y,
                 elbow_flex_right_offset_z,
                 0.0,
                 0.0, //rarm.elbowAngle,
                 0.0,
                 odomMsg.header.stamp);

    //frameid_arm_r_forearm = forearm roll DOR, at elbow pitch center
    tfs->sendEuler("forearm_roll_right",
                 "elbow_flex_right",
                 forearm_roll_right_offset_x,
                 forearm_roll_right_offset_y,
                 forearm_roll_right_offset_z,
                 0.0,
                 0.0,
                 0.0, //rarm.forearmRollAngle,
                 odomMsg.header.stamp);

    // arm_r_wrist = wrist pitch DOF.
    tfs->sendEuler("wrist_flex_right",
                 "forearm_roll_right",
                 wrist_flex_right_offset_x,
                 wrist_flex_right_offset_y,
                 wrist_flex_right_offset_z,
                 0.0,
                 0.0, //rarm.wristPitchAngle,
                 0.0,
                 odomMsg.header.stamp);

    // arm_r_hand = hand roll DOF, center at wrist pitch center
    tfs->sendEuler("gripper_roll_right",
                 "wrist_flex_right",
                 gripper_roll_right_offset_x,
                 gripper_roll_right_offset_y,
                 gripper_roll_right_offset_z,
                 0.0,
                 0.0,
                 0.0, //rarm.wristRollAngle,
                 odomMsg.header.stamp);

    // proximal digit, right
    tfs->sendEuler("finger_l_right",
                 "gripper_roll_right",
                 finger_l_right_offset_x,
                 finger_l_right_offset_y,
                 finger_l_right_offset_z,
                 0.0,  //FIXME: get angle of finger...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // proximal digit, right
    tfs->sendEuler("finger_r_right",
                 "gripper_roll_right",
                 finger_r_right_offset_x,
                 finger_r_right_offset_y,
                 finger_r_right_offset_z,
                 0.0,  //FIXME: get angle of finger...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // distal digit, right
    tfs->sendEuler("finger_tip_l_right",
                 "finger_l_right",
                 finger_tip_l_right_offset_x,
                 finger_tip_l_right_offset_y,
                 finger_tip_l_right_offset_z,
                 0.0,  //FIXME: get angle of finger tip...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // distal digit, right
    tfs->sendEuler("finger_tip_r_right",
                 "finger_r_right",
                 finger_tip_r_right_offset_x,
                 finger_tip_r_right_offset_y,
                 finger_tip_r_right_offset_z,
                 0.0,  //FIXME: get angle of finger tip...
                 0.0,
                 0.0,
                 odomMsg.header.stamp);






    // forearm camera left
    tfs->sendEuler("forearm_camera_left",
                 "forearm_roll_left",
                 forearm_camera_left_offset_x,
                 forearm_camera_left_offset_y,
                 forearm_camera_left_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // forearm camera right
    tfs->sendEuler("forearm_camera_right",
                 "forearm_roll_right",
                 forearm_camera_right_offset_x,
                 forearm_camera_right_offset_y,
                 forearm_camera_right_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // wrist camera left
    tfs->sendEuler("wrist_camera_left",
                 "gripper_roll_left",
                 wrist_camera_left_offset_x,
                 wrist_camera_left_offset_y,
                 wrist_camera_left_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // wrist camera right
    tfs->sendEuler("wrist_camera_right",
                 "gripper_roll_right",
                 wrist_camera_right_offset_x,
                 wrist_camera_right_offset_y,
                 wrist_camera_right_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);







    // head pan angle
    tfs->sendEuler("head_pan",
                 "torso",
                 head_pan_offset_x,
                 head_pan_offset_y,
                 head_pan_offset_z,
                 0.0, //FIXME: get pan angle
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // head tilt angle
    tfs->sendEuler("head_tilt",
                 "head_pan",
                 head_tilt_offset_x,
                 head_tilt_offset_y,
                 head_tilt_offset_z,
                 0.0, //FIXME: get tilt angle
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // FIXME: not implemented
    tfs->sendEuler("stereo",
                 "head_pan",
                 0.0,
                 0.0,
                 1.10,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // base laser location
    tfs->sendEuler("base_laser",
                 "base",
                 base_laser_offset_x,
                 base_laser_offset_y,
                 base_laser_offset_z,
                 0.0,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);

    // tilt laser location
    double tmpPitch, tmpPitchRate;
    //this->PR2Copy->hw.GetJointServoCmd(PR2::HEAD_LASER_PITCH, &tmpPitch, &tmpPitchRate );
    tfs->sendEuler("tilt_laser",
                 "torso",
                 tilt_laser_offset_x,
                 tilt_laser_offset_y,
                 tilt_laser_offset_z,
                 0.0,
                 tmpPitch, //FIXME: verify laser tilt angle
                 0.0,
                 odomMsg.header.stamp);


    /***************************************************************/
    // for the casters
    double tmpSteerFL, tmpVelFL;
    double tmpSteerFR, tmpVelFR;
    double tmpSteerRL, tmpVelRL;
    double tmpSteerRR, tmpVelRR;
    //this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_FL_STEER, &tmpSteerFL, &tmpVelFL );
    //this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_FR_STEER, &tmpSteerFR, &tmpVelFR );
    //this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_RL_STEER, &tmpSteerRL, &tmpVelRL );
    //this->PR2Copy->hw.GetJointServoCmd(PR2::CASTER_RR_STEER, &tmpSteerRR, &tmpVelRR );
    tfs->sendEuler("caster_front_left",
                 "base",
                 caster_front_left_offset_x,
                 caster_front_left_offset_y,
                 caster_front_left_offset_z,
                 tmpSteerFL,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_front_left_l",
                 "caster_front_left",
                 wheel_front_left_l_offset_x,
                 wheel_front_left_l_offset_y,
                 wheel_front_left_l_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_front_left_r",
                 "caster_front_left",
                 wheel_front_left_r_offset_x,
                 wheel_front_left_r_offset_y,
                 wheel_front_left_r_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);

    tfs->sendEuler("caster_front_right",
                 "base",
                 caster_front_right_offset_x,
                 caster_front_right_offset_y,
                 caster_front_right_offset_z,
                 tmpSteerFR,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_front_right_l",
                 "caster_front_right",
                 wheel_front_right_l_offset_x,
                 wheel_front_right_l_offset_y,
                 wheel_front_right_l_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_front_right_r",
                 "caster_front_right",
                 wheel_front_right_r_offset_x,
                 wheel_front_right_r_offset_y,
                 wheel_front_right_r_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);

    tfs->sendEuler("caster_rear_left",
                 "base",
                 caster_rear_left_offset_x,
                 caster_rear_left_offset_y,
                 caster_rear_left_offset_z,
                 tmpSteerRL,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_rear_left_l",
                 "caster_rear_left",
                 wheel_rear_left_l_offset_x,
                 wheel_rear_left_l_offset_y,
                 wheel_rear_left_l_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_rear_left_r",
                 "caster_rear_left",
                 wheel_rear_left_r_offset_x,
                 wheel_rear_left_r_offset_y,
                 wheel_rear_left_r_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);

    tfs->sendEuler("caster_rear_right",
                 "base",
                 caster_rear_right_offset_x,
                 caster_rear_right_offset_y,
                 caster_rear_right_offset_z,
                 tmpSteerRR,
                 0.0,
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_rear_right_l",
                 "caster_rear_right",
                 wheel_rear_right_l_offset_x,
                 wheel_rear_right_l_offset_y,
                 wheel_rear_right_l_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);
    tfs->sendEuler("wheel_rear_right_r",
                 "caster_rear_right",
                 wheel_rear_right_r_offset_x,
                 wheel_rear_right_r_offset_y,
                 wheel_rear_right_r_offset_z,
                 0.0,
                 0.0, //FIXME: get wheel rotation
                 0.0,
                 odomMsg.header.stamp);

    rosnode_->publish("transform",this->shutterMsg);
   

  }


} // namespace gazebo
