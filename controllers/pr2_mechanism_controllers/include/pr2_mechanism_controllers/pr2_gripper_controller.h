/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include <ros/node.h>
#include <mechanism_model/robot.h>
#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_effort_controller.h>
#include <pr2_mechanism_controllers/GripperControllerCmd.h>
#include <pr2_msgs/GripperControllerState.h>
#include <ethercat_hardware/PressureState.h>
#include <realtime_tools/realtime_publisher.h>

namespace controller
{
  class Pr2GripperController: public Controller
  {
    public:
      enum grasp_state {unstarted, open0, close0_closing, close0_contact, open1, close1_closing, close1_contact, complete, failed};

      Pr2GripperController();

      /*!
       * \brief Loads controller's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to this controller
       * @return Successful init
       */
      bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

      /*!
       * \brief (a) Updates commands to the gripper.
       * Called every timestep in realtime
       */
      void update();

      bool starting();

      bool stopping();

      double rampMove(double start_force, double end_force, double time, double hold);

      double stepMove(double step_size);

      double grasp();

    private:
      std::string name_;

      std::string fingertip_sensor_topic_;

      double default_speed_;

      void command_callback();

      void pressure_state_callback();

      pr2_mechanism_controllers::GripperControllerCmd grasp_cmd_desired_;

      pr2_mechanism_controllers::GripperControllerCmd grasp_cmd_;

      pr2_mechanism_controllers::GripperControllerCmd grasp_cmd;

      //ethercat_hardware::PressureState pressure_state_;  //TODO::Do I need to copy this one to make it thread safe?

      ethercat_hardware::PressureState pressure_state;

      /*!
       * \brief mutex lock for setting and getting commands
       */
      pthread_mutex_t pr2_gripper_controller_lock_;

      /*!
       * \brief true when new command received by node
       */
      bool new_cmd_available_;

      /*!
       * \brief timeout specifying time that the controller waits before setting the current velocity command to zero
       */
      double timeout_;

      /*!
       * \brief time corresponding to when update was last called
       */
      double last_time_;

      /*!
      * \brief timestamp remembering when the last command was received
      */
      double cmd_received_timestamp_;

      /*!
      * \brief amplitude in joint effort values used to break stiction
      */
      double break_stiction_amplitude_;

      /*!
       * \brief duration in seconds used to increment effort or period of effort's overlaid sine wave
       */
      double break_stiction_period_;

      /*!
       * \brief velocity at which stiction is consitered broken
       */
      double break_stiction_velocity_;

      /*!
       * \brief type of stiction breaking: none, sine, ramp
       */
      std::string break_stiction_type_;

      /*!
       * \brief last commanded command stripped of any break stiction forces
       */
      double last_commanded_command;

      /*!
       * \brief offset from desired value where the moveTo command goes from full force to linear
       */
      double proportional_offset_;

      /*!
       * \brief remembers last impact time
       */
      double grasp_impact_timestamp;

      /*!
       * \brief remembers last time it was commanded to close or open
       */
      ros::Time grasp_open_close_timestamp;

      /*!
       * \brief remembers last time it was commanded to close or open
       */
      ros::Duration timeout_duration;

      /*!
       * \brief remembers the state of the closed loop grasp
       */
      grasp_state closed_loop_grasp_state;

      /*!
       * \brief remembers everything about the state of the robot
       */
      mechanism::RobotState *robot_state_;

      /*!
       * \brief JointState for this caster joint
       */
      mechanism::JointState *joint_;

      JointEffortController joint_controller_;

      /*!
       * \brief The maximum value that can be counted as stopped for closed loop grasping
       */
      double stopped_threshold_;

      double parseMessage(pr2_mechanism_controllers::GripperControllerCmd desired_msg);

      double effortLimit(double desiredEffort);

      /*!
       * \brief publishes information about the caster and wheel controllers
       */
      realtime_tools::RealtimePublisher<pr2_msgs::GripperControllerState>* state_publisher_;

      int fingertip_sensor_start0[15];
      int fingertip_sensor_start1[15];
      int fingertip_sensor_first_peak0[15];
      int fingertip_sensor_first_peak1[15];
      int fingertip_sensor_first_steady0[15];
      int fingertip_sensor_first_steady1[15];
      int fingertip_sensor_second_peak0[15];
      int fingertip_sensor_second_peak1[15];
      int fingertip_sensor_second_steady0[15];
      int fingertip_sensor_second_steady1[15];

      double position_first_contact;
      double position_second_contact;
      double position_first_compression;
      double position_second_compression;
      int peak_force_first_grasp;
      int peak_force_second_grasp;

      double low_force_;
      double high_force_;

      double spring_const;

      int contact_threshold_;
  };
}

