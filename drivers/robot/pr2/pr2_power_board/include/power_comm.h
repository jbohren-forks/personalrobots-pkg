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
;5B * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POWER_COMM_H
#define POWER_COMM_H

static const unsigned CURRENT_MESSAGE_REVISION = 1;
static const unsigned SERIAL_NUMBER = 1;
static const int MAX_PUMP_TRYS = 30;
static const unsigned STATUS_MESSAGE_TIMER = 4688;  //100Hz at 46KHz timer
static const float THRESHOLD_18V_GOOD = 17.0; //low limit for 18v standby power
static const float THRESHOLD_48V_GOOD = 28.0; //low limit for input voltage
static const float THRESHOLD_HYST = 2.0; // provide some hysteriesis
static const int TEMP_LIMIT_MAX = 50; // 90 degrees celcius limit
static const unsigned POWER_PORT = 6801; // port power board
										 // broadcasts status data on

enum Master_State { MASTER_NOPOWER, MASTER_STANDBY, MASTER_ON, MASTER_OFF };
enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
enum CB_Command { NONE = 0, COMMAND_START = 1, COMMAND_STOP = 2, COMMAND_RESET = 3, COMMAND_DISABLE = 4 };

typedef struct
{
  unsigned int    message_revision; //32 bit 
  unsigned int    serial_num;       //32 bit  Unique ID number
  char            text[32];         //Description identifier
  unsigned int    dumy;
  unsigned int    data_length;      //Length of the following structure
} __attribute__((__packed__)) MessageHeader;

typedef struct
{
  //Software State  0 = default, 1=Start, 2=Stop, 3=reset, 4=disable
  unsigned char   CB0_state;   // CB_State enum
  unsigned char   CB1_state;
  unsigned char   CB2_state;
  unsigned char   DCDC_state;  // Master_State enum

  //Status
  float           input_voltage;
  float           input_current;
  float           DCDC_12V_out_voltage;
  float           DCDC_19V_out_voltage;
  float           CB0_voltage;
  float           CB1_voltage;
  float           CB2_voltage;
  float           ambient_temp;
  unsigned int    fan0_speed; 
  unsigned int    fan1_speed;
  unsigned int    fan2_speed;
  unsigned int    fan3_speed;
  unsigned char   CB0_status;  //0-off 1-on
  unsigned char   CB1_status;
  unsigned char   CB2_status;
  unsigned char   estop_button_status;
  unsigned char   estop_status;
  unsigned char   pca_rev;
  unsigned char   pcb_rev;
  unsigned char   major_rev;
  unsigned char   minor_rev;

} __attribute__((__packed__)) StatusStruct;

typedef struct 
{
	MessageHeader header;
	StatusStruct  status;
} __attribute__((__packed__)) PowerMessage;

typedef struct
{
  unsigned char   CB0_command; //CB_Command enum
  unsigned char   CB1_command;
  unsigned char   CB2_command;
  unsigned char   DCDC_command;
  unsigned char   fan0_command;
  unsigned char   fan1_command;
  unsigned char   fan2_command;
  unsigned char   fan3_command;
  unsigned int    reserved;
} __attribute__((__packed__)) CommandStruct;

typedef struct
{
  MessageHeader header;
  CommandStruct command;
} __attribute__((__packed__)) CommandMessage;

#endif
