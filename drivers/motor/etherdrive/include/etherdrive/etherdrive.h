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

/*! \mainpage Etherdrive Driver
 *  \htmlinclude manifest.html
 */

#ifndef ETHERDRIVE_H
#define ETHERDRIVE_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <string>

using namespace std;

class EDMotor;

/*! \brief A simple class to interface to the Ethedrive board.
 *
 * The EtherDrive class interfaces to the EtherDrive board via an
 * underlying UDP transport layer.  It provides control of up to
 * 6 motors.
 * 
 * Motor properties can be set either directly via motor number, or
 * via the EDMotor helper class.
 * 
 * In order to send commanded drive values to the etherdrive board and
 * retrieve encoder values, the EtherDrive::tick method must be
 * called at a sufficient rate or else the motors will be disabled.
 */
class EtherDrive
{
public:
  /*! The constructor does very little of interest.  After
   * construction, EtherDrive::init must be called before the
   * EtherDrive can be used.
   */
  EtherDrive();
  /*! The deconstructor will shutdown the UDP sockets if necessary. */
  ~EtherDrive();

  /*! Connects up to the EtherDrive board located at the given IP.
   * 
   * \param ip  The IP address of the EtherDrive board to connect to.
   */
  bool init(string ip);
  
  /*! Close the UDP sockets */
  void shutdown();

  /*! Manually send an EtherDrive command.
   * The commands are documented in the EtherDrive manual at: http://hubbard.engr.scu.edu/embedded/motorcontrol/etherdrive/v20/
   *
   * \param cmd     Pointer to the command buffer.
   * \param cmd_len Length of the command buffer.
   * \param buf     Pointer to the response buffer.
   * \param buf_len Length of the response buffer
   * \return The length of the reponse received or -1 if there was a failure.
   */
  int send_cmd(char* cmd, size_t cmd_len, char* buf, size_t buf_len);

  /*! Send command to turn on all motors */
  bool motors_on();

  /*! Send command to turn off all motors */
  bool motors_off();

  /*! Sets the control mode
   *
   * \param m  The mode type.  0 = voltage, 1 = current, 2 = position
   */
  bool set_control_mode(int8_t m);

  /*! Sets the gain values
   *
   * \param m  The motor number (0-5).
   * \param G  A character for the gain-type to be set:
   * P = Proportional,
   * I = Integral,
   * D = Derivative,
   * W = Windmax,
   * M = Max output,
   * Z = Deadband
   * \param val  The value to set the given gain to.
   */
  bool set_gain(uint32_t m, char G, int32_t val);

  /*! Returns an EDMotor helper.
   * \param m  The motor number (0-5)..
   * \returns An EDMotor instance.
   */
  EDMotor get_motor(uint32_t m);

  /*! Sets the drive value of a motor.
   * \param m  The motor number (0-5).
   * \param drv  The drive value.
   */
  void set_drv(uint32_t m, int32_t drv);

  /*! Returns the encoder value of a motor.
   * \param m  The motor number (0-5).
   * \returns The encoder value.
   */
  int32_t get_enc(uint32_t m);

  /*! Returns the current value of a motor.
   * \param m  The motor number (0-5).
   * \returns The current value.
   */
  int32_t get_cur(uint32_t m);

  /*! Returns the pwm value of a motor
   * \param m  The motor number (0-5)..
   * \returns The pwm value.
   */
  int32_t get_pwm(uint32_t m);

  /*! Sets the drive values of multiple motors at once.  All motors >=
   *  num will not be changed.
   *
   * \param num  The number of motors to set (1-6)
   * \param drv  An array of values of length num.
   */
  bool drive(size_t num, int32_t* drv);

  /*! Send most recent motor commands, and retrieve updates.  This
   * command must be run at a sufficient rate or else the motors will
   * be disabled.
   *
   * \param num (optional) the number of motors to retrieve status for
   * \param enc (optional) an array of length num where encoder values will be stored
   * \param curr (optional) an array of length num where current values will be stored
   * \param pwm (optional) an array of length num where pwm values will be stored
   */
  bool tick(size_t num = 0, int32_t* enc = 0, int32_t* curr = 0, int32_t* pwm = 0);

private:
  bool ready; /*!< Whether or not initialize has been run successfully*/

  int32_t last_drv[6];  /*!< Last set drive values */
  int32_t last_enc[6];  /*!< Last returned encoder values */
  int32_t last_cur[6];  /*!< Last returned current values */
  int32_t last_pwm[6];  /*!< Last returned pwm values */

  int mot_sock;   /*!< Socket for sending/receiving motor drive commands. */
  int cmd_sock;   /*!< Socket for sending/receiving configuration commands */

  struct sockaddr_in mot_addr_out;
  struct sockaddr_in cmd_addr_out;
};

/*! \brief A helper class to provide access to a particular motor.
 *
 * The EDMotor helper class must be allocated via a call to
 * EtherDrive::get_motor.  It will keep track of its motor number and
 * pass through commands to the EtheDrive that allocated it.
 */
class EDMotor
{
  friend class EtherDrive;
public:
  /*! Sets the drive value of a motor.  Passes through to EtherDrive::set_drv but populates motor number.*/
  void set_drv(int32_t drv) {
    driver->set_drv(motor, drv);
  }

  /*! Retrieves the encoder value of a motor.  Passes through to EtherDrive::get_enc but populates motor number.*/
  int32_t get_enc() {
    return driver->get_enc(motor);
  }

  /*! Retrieves the current value of a motor.  Passes through to EtherDrive::get_cur but populates motor number.*/
  int32_t get_cur() {
    return driver->get_cur(motor);
  }

  /*! Retrieves the pwm value of a motor.  Passes through to EtherDrive::get_pwm but populates motor number.*/
  int32_t get_pwm() {
    return driver->get_pwm(motor);
  }

  /*! Sets the gain values for the motor.
   * \param P  Proportional gain.
   * \param I  Integral gain.
   * \param D  Derivative gain.
   * \param W  Maximum integral windup.
   * \param M  Maximum output.
   * \param Z  Deadband.
   */
  bool set_gains(int32_t P, int32_t I, int32_t D, int32_t W, int32_t M, int32_t Z) {
    return driver->set_gain(motor, 'P', P) & 
      driver->set_gain(motor, 'I', I) & 
      driver->set_gain(motor, 'D', D) & 
      driver->set_gain(motor, 'W', W) & 
      driver->set_gain(motor, 'M', M) & 
      driver->set_gain(motor, 'Z', Z);
  }
protected:
  /*! Constructor is protected so that it can only be allocated by EtherDrive */
  EDMotor(EtherDrive* driver, uint32_t motor) : driver(driver), motor(motor) {}

private:
  EtherDrive* driver; /*!< Pointer to the EtherDrive parent*/

  uint32_t motor; /*!< Motor number*/
};


#endif

