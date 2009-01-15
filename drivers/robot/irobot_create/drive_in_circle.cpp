#include "irobot_create/irobot_create.h"

int main(int, char **)
{
  IRobotCreate robot;
  robot.Start();
  int left = 0; int right =0;
  robot.getReadings(left, right);
//  robot.getAngle();

  
  //double angle = 0.001; double velocity = 0.3;
  double l_vel = 0.4; double r_vel = 0.4;
/*  
  while (true) {
  sleep(0.5);
  robot.getReadings(left, right);
  if (left ==1 && right ==1) {robot.setVelocity(-velocity, angle); sleep(1); robot.setVelocity(0.01, 1.57);}
  else if (left ==1 && right ==0) {robot.setVelocity(0.01, 1.57); sleep(1);}
  else if (left ==0 && right ==1) {robot.setVelocity(0.01, -1.57); sleep(1);}
  while (angle>3.14) angle = angle-3.14;
  while (angle<-3.14) angle = angle+3.14;
  robot.setVelocity(velocity, angle);
//  robot.getAngle();
  cout << endl;
  }
*/

	while (true) {
		robot.setWheels(l_vel, r_vel);
		robot.getReadings(left, right);
		if (left ==1 && right ==1) {robot.setWheels(-l_vel, -r_vel); sleep(2); robot.setWheels(0.00, 0.3); sleep (1); }
		else if (left ==1 && right ==0) {robot.setWheels (-l_vel, -r_vel); sleep(2); robot.setWheels(0.3, 0.0); sleep (1);}
		else if (left ==0 && right ==1) {robot.setWheels(-l_vel, -r_vel); sleep(2); robot.setWheels(0.00, 0.3); sleep (1);}
		sleep(0.5);
		cout << endl;
	}
    robot.setVelocity(0.0, 0.0);
  return 0;
}

