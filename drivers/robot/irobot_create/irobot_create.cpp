#include <cstdlib>
#include "rosconsole/rosconsole.h"
#include "irobot_create/irobot_create.h"

IRobotCreate::IRobotCreate()
{
  // see if the IROBOT_CREATE_OPPORT variable is defined
  const char *default_port = "/dev/ttyUSB0";
  const char *port_env = getenv("IROBOT_CREATE_PORT");
  const char *serial_port_str = (port_env ? port_env : default_port);
  ROS_DEBUG("irobotcreate ctor, serial_port_str = [%s]", serial_port_str);
  serial_port = new LightweightSerial(serial_port_str, 57600);
}

IRobotCreate::~IRobotCreate()
{
  ROS_DEBUG("irobotcreate dtor");
  ROS_DEBUG("closing serial port");
  delete serial_port;
  serial_port = NULL;
  ROS_DEBUG("irobotcreate dtor complete");
}
void IRobotCreate::Wait(double useconds) {
/*
	time_t start,end;  
        time (&start);
	while	(true) {
		time (&end);
		double dif = difftime (end, start);
		if (dif >=seconds) break;
	}
*/
	usleep(useconds);
}

void IRobotCreate::Wait() {
	usleep(CREATE_DELAY_MODECHANGE_MS);
}

bool IRobotCreate::Start() {

//	Wait();
	serial_port->write((uint8_t)CREATE_OPCODE_START);
	Wait();
	
//	int usleep(useconds_t useconds);

//	Wait(1.0);
	serial_port->write((uint8_t)CREATE_OPCODE_FULL);
	Wait();
//	Wait(1.0);
	ROS_DEBUG("irobotcreate started");
//	serial_port->write((uint8_t)CREATE_OPCODE_COVER);usleep(20);
//	Wait(1.0);
//	ROS_DEBUG("here?");
	return true;
}


double IRobotCreate::getReadings(int &left, int &right)
{
	uint8_t input1, input2;
//	serial_port->read(&input1);
//	Wait();
	
	serial_port->write((uint8_t)CREATE_OPCODE_QUERY_LIST);
	Wait();
	uint8_t number = 5;
	serial_port->write((uint8_t)number);	Wait();
	serial_port->write((uint8_t)CREATE_REQUESTED_VELOCITY); Wait();
	serial_port->write((uint8_t)CREATE_REQUESTED_RADIUS); Wait();
	serial_port->write((uint8_t)CREATE_SENSOR_DISTANCE); Wait();
	serial_port->write((uint8_t)CREATE_SENSOR_ANGLE); Wait();
	serial_port->write((uint8_t)CREATE_SENSOR_BUMPER); Wait();

	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("velocity requested = %d %d mm/s", input1, input2);
	ROS_DEBUG("velocity requested = %d mm/s",  toDecimal(input1 * 256+ input2));
	
	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("radius requested = %d %d mm", input1, input2);
	ROS_DEBUG("radius requested = %d mm",  toDecimal(input1 * 256+ input2));
	
	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("distance traveled = %d %d mm", input1, input2);	
	ROS_DEBUG("distance traveled = %d mm",  toDecimal(input1 * 256+ input2));

	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("angle changed = %d %d degree", input1, input2);	
	ROS_DEBUG("angle changed = %d degree", toDecimal(input1 * 256+ input2));

	serial_port->read(&input1);
	Wait();
	left =input1 &1;
	right = (input1>>1) & 1;
	ROS_DEBUG("bumper %d", input1);
	ROS_DEBUG("bumper right %d left %d", left, right);
	
/*
//	serial_port->write((uint8_t)CREATE_OPCODE_SENSORS);
//	Wait();
	serial_port->write((uint8_t)CREATE_SENSOR_DISTANCE);
//	uint8_t input1, input2;
	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("distance traveled = %d %d mm", input1, input2);	
	ROS_DEBUG("distance traveled = %d mm",  toDecimal(input1 * 256+ input2));
*/	return 1.0;

}

int IRobotCreate::toDecimal(int decimal) {
	int bin[16]; int sign = 1;
/*	ROS_DEBUG("%d",decimal);
	sprintf( hex, "%08x", decimal);
	ROS_DEBUG("%s",hex);
	int i = atoi(hex);
	ROS_DEBUG("%d",i);
	sprintf(dec, "%08x", i);
	ROS_DEBUG("%s",dec);
	int j = atoi(dec);
	ROS_DEBUG("%d",j);
	*/
//	ROS_DEBUG("%d",decimal);
//	itoa(decimal, bin, 2);	
	for (int i = 15; i >=0; i--) {
		bin[i] = decimal % 2;
		decimal = decimal / 2;
//		cout << ' ' << bin[i];
	}
	if (bin[0] == 1) {
		int pos = 15; sign = -1;
		while (bin[pos] == 0 && pos>=0) pos--;
		for (int i = 0; i< pos; i++) 
			bin[i] = 1 - bin[i];
	}
	int sum = 0; int factor = 1;
	for (int i = 15; i >= 0; i--) {
//		cout << ' ' << bin[i];
		sum +=bin[i] * factor;
		factor *=2;
	}
	return sum*sign;
}
/*
double IRobotCreate::getAngle()
{
	serial_port->write((uint8_t)CREATE_OPCODE_SENSORS);
	Wait();
	serial_port->write((uint8_t)CREATE_SENSOR_ANGLE);
	uint8_t input1, input2;
	serial_port->read(&input1);
	Wait();
	serial_port->read(&input2);
	Wait();
	ROS_DEBUG("angle changed = %d %d degree", input1, input2);	
	ROS_DEBUG("angle changed = %d degree", toDecimal(input1 * 256+ input2));
  return 1.0;
}
*/
bool IRobotCreate::setVelocity(double x_vel, double w_vel)
{
//convert linear (m/s) and angular velocity (rad/s) into velocity (mm/s) and radius (mm)

	int velocity = ((int)(x_vel * 1000.0));
	if (velocity > CREATE_TVEL_MAX_MM_S) velocity = CREATE_TVEL_MAX_MM_S;
		else if (velocity <-CREATE_TVEL_MAX_MM_S) velocity = -CREATE_TVEL_MAX_MM_S;
	
	int radius;
	if (w_vel == 0.0) radius = CREATE_RADIUS_MAX_MM;
	else radius = ((int)(x_vel / w_vel * 1000.0));
	if (radius > CREATE_RADIUS_MAX_MM) radius = CREATE_RADIUS_MAX_MM;
		else if (radius <-CREATE_RADIUS_MAX_MM) radius = -CREATE_RADIUS_MAX_MM;

	int high1, low1, high2, low2;
	toOpcode(velocity, high1, low1);
	toOpcode(radius, high2, low2);
	
	
	serial_port->write((uint8_t)CREATE_OPCODE_DRIVE);
	Wait();
	serial_port->write((uint8_t)high1);
	Wait();
	serial_port->write((uint8_t)low1);
	Wait();
	serial_port->write((uint8_t)high2);
	Wait();
	serial_port->write((uint8_t)low2);
	Wait();


//	serial_port->write((uint8_t)CREATE_OPCODE_COVER);
//	Wait();
	ROS_DEBUG("setting velocity to %d mm/s, radius %d mm", velocity, radius);
	ROS_DEBUG("setting velocity to code [%d][%d], [%d][%d]", high1, low1, high2, low2);
	return true;
}

bool IRobotCreate::setWheels(double l_vel, double r_vel)
{
//left and right wheel velocities (in mm/s)
	int left = l_vel *1000; int right = r_vel*1000;
	if (left > CREATE_TVEL_MAX_MM_S) left= CREATE_TVEL_MAX_MM_S;
	else if (left < -CREATE_TVEL_MAX_MM_S) left= -CREATE_TVEL_MAX_MM_S;
	if (right > CREATE_TVEL_MAX_MM_S) right= CREATE_TVEL_MAX_MM_S;
	else if (right < -CREATE_TVEL_MAX_MM_S) right= -CREATE_TVEL_MAX_MM_S;
	
	int high1, low1, high2, low2;
	toOpcode(right, high1, low1);
	toOpcode(left, high2, low2);
	
	
	serial_port->write((uint8_t)CREATE_OPCODE_DRIVE_WHEELS);
	Wait();
	serial_port->write((uint8_t)high1);
	Wait();
	serial_port->write((uint8_t)low1);
	Wait();
	serial_port->write((uint8_t)high2);
	Wait();
	serial_port->write((uint8_t)low2);
	Wait();


//	serial_port->write((uint8_t)CREATE_OPCODE_COVER);
//	Wait();
	ROS_DEBUG("setting velocity to left %d mm/s, right %d mm/s", left, right);
	ROS_DEBUG("setting velocity to code [%d][%d], [%d][%d]", high1, low1, high2, low2);
	return true;
}
void IRobotCreate::toOpcode(int decimal, int & high, int & low) {
	char hex[8];
	sprintf( hex, "%08x", decimal);
//	for (int i = 0; i < 8; i++ ) {
//		printf ("%c", hex[i]);
//	}
	int first, second;
	if (hex[4]>='0' && hex[4]<='9')	{ first = hex[4] - '0'; }
		else if (hex[4]>='a' && hex[4]<='f')  { first = hex[4] - 'a' +10; }
	if (hex[5]>='0' && hex[5]<='9')	{ second = hex[5] - '0'; }
		else if (hex[5]>='a' && hex[5]<='f')  { second = hex[5] - 'a' +10; }
	high = first *16 + second;
	if (hex[6]>='0' && hex[6]<='9')	{ first = hex[6] - '0'; }
		else if (hex[6]>='a' && hex[6]<='f')  { first = hex[6] - 'a' +10; }
	if (hex[7]>='0' && hex[7]<='9')	{ second = hex[7] - '0'; }
		else if (hex[7]>='a' && hex[7]<='f')  { second = hex[7] - 'a' +10; }
	low = first *16 + second;
}
