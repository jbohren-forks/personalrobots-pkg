/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef ETHERCAT_HARDWARE_H
#define ETHERCAT_HARDWARE_H

#include <hw_interface/hardware_interface.h>

#include <al/ethercat_AL.h>
#include <al/ethercat_master.h>
#include <al/ethercat_slave_handler.h>

#include "ethercat_hardware/motor_control_board.h"

class EthercatHardware {
public:
   /*!
    * \brief Constructor
    */
    EthercatHardware();

   /*!
    * \brief Destructor
    */
   ~EthercatHardware();

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
    * \brief Initialize the etherdrive interface by calling init() on all the etherdrive boards. The boards are set to current control and the gains for the boards are also set up here.
    */
   void init(char *interface, HardwareInterface *hw);


private:
    struct netif *ni;
    HardwareInterface *hw;

    EtherCAT_AL *al;
    EtherCAT_Master *em;

    MotorControlBoard *configSlave(EtherCAT_SlaveHandler *sh);
    MotorControlBoard **slaves;
    unsigned int numSlaves;

    unsigned char *buffer;
    unsigned int bufferSize;
};

#endif /* ETHERCAT_HARDWARE_H */
