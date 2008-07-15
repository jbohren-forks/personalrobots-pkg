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

//Etherdrive hardware implementation
//Creates etherdrive interface to the robot actuators

#ifndef ETHERDRIVE_HARDWARE_H
#define ETHERDRIVE_HARDWARE_H

#include "etherdrive/etherdrive.h"
#include "hw_interface/hardware_interface.h"
#include <string>

#define MAX_NUM_ACTUATORS 64
const float ETHERDRIVE_CURRENT_TO_CMD = 2000;

class EtherdriveHardware{
   /*! \class 
    * \brief Create an etherdrive interface to the robot hardware
    */
  public:

   /*! 
    * \brief Constructor that initializes all the etherdrive boards
    * \param int numBoards - number of boards hooked up in the robot
    * \param int numActuators - total number of actuators hooked into the boards
    * \param int[] boardLookUp - array of integers specifying board numbers for the actuators 
    * \param int[] portLookUp - array of integers specifying port numbers on the board for each actuator
    * \param int[] jointId - array of jointIds to allow lookup into hardware interface
    * \param string[] etherIP - array of IPs for each board
    * \param string[] hostIP - array of IPs for each host that the board is hooked up to
    */
  EtherdriveHardware(int numBoards, int numActuators, int boardLookUp[], int portLookUp[], int jointId[], string etherIP[], string hostIP[], HardwareInterface *hw);

   /*!
    * \brief Destructor
    */
   ~EtherdriveHardware();

   /*! 
    * \brief Update the state (in the hardware interface) by reading the values from the encoders
    \param HardwareInterface* hw pointer to the hardware interface
   */
   void updateState();

   /*! 
    * \brief Update send most recent motor commands and retrieve updates. This command must be run at a sufficient rate or else the motors will be disabled.
   */
   void update();

   /*! 
    * \brief Read the command values from the hardware interface and send them out to the actual motors
    \param HardwareInterface* hw pointer to the hardware interface
   */
   void sendCommand();

   /*! 
    * \brief Initialize the etherdrive interface by calling init() on all the etherdrive boards. The boards are set to current control and the gains for the boards are also set up here.
    */
   void init();

  private:

   /*! 
    * \brief Setup the gains for the etherdrive boards 
    */
   void setGains(int P, int I, int D, int W, int M, int Z);
       
   /*! 
    * \brief Setup the control mode for the etherdrive boards 
    */
   void setControlMode(int controlMode);

  /*! 
    * \brief Send command to turn all the motors on or off on the etherdrive boards 
    */
   void setMotorsOn(bool motorsOn);

   int numBoards; /**< number of etherdrive boards on the robot */

   int numActuators; /**< number of actuators */

   int boardLookUp[MAX_NUM_ACTUATORS]; /**< array of board ids for the actuators */

   int portLookUp[MAX_NUM_ACTUATORS]; /**< array of port ids for the actuators */

   int jointId[MAX_NUM_ACTUATORS]; /**< array of joint ID for each of the actuators */

   string etherIP[MAX_NUM_ACTUATORS]; /**< ethernet address of etherdrive module for the actuator */

   string hostIP[MAX_NUM_ACTUATORS]; /**< host address of etherdrive module for the actuator */

   EtherDrive *edBoard; /**< pointer to the hardware object */

   HardwareInterface *hw;

   enum ETHERDRIVE_CONTROL_MODE{
      ETHERDRIVE_VOLTAGE_MODE,
      ETHERDRIVE_CURRENT_MODE
   };
};

#endif
