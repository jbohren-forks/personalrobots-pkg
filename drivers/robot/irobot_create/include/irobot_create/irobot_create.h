#ifndef IROBOT_CREATE_H
#define IROBOT_CREATE_H

#include "rosconsole/rosconsole.h"
#include "serial_port/lightweightserial.h"
#include <time.h>
#include <cstdio>
#include <iostream>
#include <unistd.h>


using namespace std;
class IRobotCreate
{
public:
  IRobotCreate();
  ~IRobotCreate();

	double getReadings(int & left, int  &right); //distance traveled in mm, angle turned in degree
//	double getAngle(); //in degree
	bool setVelocity(double x_vel, double w_vel); // linear, angular velocities	
	bool setWheels(double l_vel, double r_vel); //left and right wheel velocities
	bool Start();
    LightweightSerial *serial_port;
private:
  void Wait();		//20 us
  void Wait(double useconds);
  void toOpcode(int decimal, int & high, int & low); //convert int to 2 uint8_t
  int toDecimal(int decimal); //convert 2 uint8_t to int
	
};
//1 byte = 8 bits
#define CREATE_OPCODE_START            128
#define CREATE_OPCODE_BAUD             129
#define CREATE_OPCODE_SAFE             131
#define CREATE_OPCODE_FULL             132
#define CREATE_OPCODE_SPOT             134
#define CREATE_OPCODE_COVER            135
#define CREATE_OPCODE_DEMO             136
#define CREATE_OPCODE_DRIVE            137
#define CREATE_OPCODE_MOTORS           138
#define CREATE_OPCODE_LEDS             139
#define CREATE_OPCODE_SONG             140
#define CREATE_OPCODE_PLAY             141
#define CREATE_OPCODE_SENSORS          142
#define CREATE_OPCODE_COVERDOCK        143
#define CREATE_OPCODE_PWM_MOTORS       144
#define CREATE_OPCODE_DRIVE_WHEELS     145
#define CREATE_OPCODE_DIGITAL_OUTPUTS  147
#define CREATE_OPCODE_STREAM           148
#define CREATE_OPCODE_QUERY_LIST       149
#define CREATE_OPCODE_DO_STREAM        150
#define CREATE_OPCODE_SEND_IR_CHAR     151
#define CREATE_OPCODE_SCRIPT           152
#define CREATE_OPCODE_PLAY_SCRIPT      153
#define CREATE_OPCODE_SHOW_SCRIPT      154
#define CREATE_OPCODE_WAIT_TIME        155
#define CREATE_OPCODE_WAIT_DISTANCE    156
#define CREATE_OPCODE_WAIT_ANGLE       157
#define CREATE_OPCODE_WAIT_EVENT       158


#define CREATE_DELAY_MODECHANGE_MS      5000

#define CREATE_MODE_OFF                  0
#define CREATE_MODE_PASSIVE              1
#define CREATE_MODE_SAFE                 2
#define CREATE_MODE_FULL                 3

#define CREATE_TVEL_MAX_MM_S           500     
#define CREATE_RADIUS_MAX_MM          2000

#define CREATE_SENSOR_PACKET_SIZE       26

#define CREATE_CHARGING_NOT              0
#define CREATE_CHARGING_RECOVERY         1
#define CREATE_CHARGING_CHARGING         2
#define CREATE_CHARGING_TRICKLE          3
#define CREATE_CHARGING_WAITING          4
#define CREATE_CHARGING_ERROR            5
#define CREATE_SENSOR_BUMPER           7
#define CREATE_SENSOR_DISTANCE		19
#define CREATE_SENSOR_ANGLE		20
#define CREATE_REQUESTED_VELOCITY		39
#define CREATE_REQUESTED_RADIUS		40
#define CREATE_REQUESTED_RVELOCITY		41
#define CREATE_REQUESTED_LVELOCITY		42

#define CREATE_AXLE_LENGTH            0.258

#define CREATE_DIAMETER 0.33

#define CREATE_BUMPER_XOFFSET 0.05

#endif

