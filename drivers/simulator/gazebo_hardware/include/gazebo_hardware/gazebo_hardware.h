///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Sachin Chitta, Jimmy Sastra
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

//Simulated Iface-hardware implementation for gazebo

#ifndef GAZEBO_HARDWARE_H
#define GAZEBO_HARDWARE_H

#include <pr2Core/pr2Core.h>
#include <pr2Core/pr2Misc.h>
#include <math.h>
#include <list>
#include <vector>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

using namespace gazebo;
using namespace std;

#include "hw_interface/hardware_interface.h"
#include <string>

#define GAZEBO_CURRENT_TO_CMD 1.0
#define GAZEBO_POS_TO_ENCODER 1000000.0
#define MAX_NUM_ACTUATORS 1000

class GazeboHardware{
  /*! \class 
   * \brief Create an gazebo interface to the robot simulation
   */
 public:

  /*! 
   * \brief Constructor that initializes all the etherdrive boards (Pr2_Actarrays)
   * \param int numBoards - number of boards hooked up in the robot (number of Pr2_Actarrays)
   * \param int numActuators - total number of actuators hooked into the boards (number of actuators for each Pr2_Actarray)
   * \param int[] boardLookUp - array of integers specifying board numbers for the actuators
   * \param int[] portLookUp - array of integers specifying port numbers on the board for each actuator
   * \param int[] jointId - array of jointIds to allow lookup into hardware interface
   * \param string[] etherIP - array of IPs for each board (TBD)
   * \param string[] hostIP - array of IPs for each host that the board is hooked up to (TBD)
   */
 GazeboHardware(int numBoards, int numActuators, int boardLookUp[], int portLookUp[], int jointId[], string etherIP[], string hostIP[]);

  /*!
   * \brief Destructor
   */
  ~GazeboHardware();

  /*! 
   * \brief Update the state (in the hardware interface) by reading the values from the encoders
   \param HardwareInterface* hw pointer to the hardware interface
  */
  void updateState();

  /*! 
   * \brief tick send most recent motor commands and retrieve updates. This command must be run at a sufficient rate or else the motors will be disabled.
  */
  void tick();

  /*! 
   * \brief Read the command values from the hardware interface and send them out to the actual motors
   \param HardwareInterface* hw pointer to the hardware interface
  */
  void sendCommand();

  /*! 
   * \brief Initialize the gazebo iface interface by calling init() on all the actarrays.
   * The boards are set to current control and the gains for the boards are also set up here.
   */
  void init();

 private:

  /*! 
   * \brief Setup the gains for the gazebo controllers 
   */
  void setGains(int P, int I, int D, int W, int M, int Z);
      
  /*! 
   * \brief Setup the control mode for the gazebo controllers 
   */
  void setControlMode(int controlMode);

  /*! 
    * \brief Send command to turn all the motors on or off on the gazebo actuator arrays
    */
  void setMotorsOn(bool motorsOn);

  int numBoards; /**< number of actuator arrays on the robot */

  int numActuators; /**< number of actuators */

  int boardLookUp[MAX_NUM_ACTUATORS]; /**< array of board ids for the actuators */

  int portLookUp[MAX_NUM_ACTUATORS]; /**< array of port ids for the actuators */

  int jointId[MAX_NUM_ACTUATORS]; /**< array of joint ID for each of the actuators */

  string etherIP[MAX_NUM_ACTUATORS]; /**< TBD for simulation (ethernet address of etherdrive module for the actuator) */

  string hostIP[MAX_NUM_ACTUATORS]; /**< TBD for simulation (host address of etherdrive module for the actuator) */

  ////////////////////////////////////////////////////////////////////
  //                                                                //
  //  Gazebo Client Interfaces                                      //
  //  these are the "hardware" interfaces                           //
  //                                                                //
  ////////////////////////////////////////////////////////////////////
  gazebo::Client           *client;  /**< connect to simulation as a client */
  gazebo::PR2ArrayIface    *pr2ActarrayIface; /**< pointer to the actuator actarrays */
  gazebo::PR2GripperIface  *pr2GripperLeftIface; /**< pointer to left gripper iface */
  gazebo::PR2GripperIface  *pr2GripperRightIface; /**< pointer to right gripper iface */
  gazebo::PTZIface         *pr2PTZCameraLeftIface; /**< pointer to left gripper iface */
  gazebo::PTZIface         *pr2PTZCameraRightIface; /**< pointer to right gripper iface */

  HardwareInterface *hw; /**< a class holding an array of actuators */

  enum GAZEBO_CONTROL_MODE{
       GAZEBO_VOLTAGE_MODE,
       GAZEBO_CURRENT_MODE
  };

};

#endif
