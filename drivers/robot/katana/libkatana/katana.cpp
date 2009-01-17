#include <cstdio>
#include <stdexcept>
#include <vector>
#include "katana/katana.h"
#include "ros/common.h"
#include <iostream>
#include <math.h>

using std::string;
using std::vector;
using std::endl;
using std::cout;

#define PI 3.14159265358979

Katana::Katana()
{
  // this constructor will throw exceptions if it doesn't work
  // snarf in the text file containing the serial port string
  string katana_pkg_path = ros::getPackagePath("katana");
  if (!katana_pkg_path.length())
    throw std::runtime_error("couldn't see package 'katana' using rospack");
  string serial_port_file = katana_pkg_path + "/cfg/serial_port";
  FILE *f = fopen(serial_port_file.c_str(), "r");
  if (!f)
  {
    // write this file if it doesn't already exist
    f = fopen(serial_port_file.c_str(), "w");
    if (!f)
      throw std::runtime_error("couldn't create serial port config file");
    fprintf(f, "/dev/ttyUSB0"); // guess
    fclose(f);
    f = fopen(serial_port_file.c_str(), "r");
    if (!f)
      throw std::runtime_error("couldn't re-open serial port config file");
  }
  char serial_port_cstr[200] = "/dev/ttyS0"; // something reasonable for guess
  fscanf(f, "%200s", serial_port_cstr);
  fclose(f);
  TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, 300, 0, string(serial_port_cstr)};
  device = new CCdlCOM(ccd);
  printf("opened serial port [%s] successfully\n", serial_port_cstr);
  protocol = new CCplSerialCRC();
  protocol->init(device);
  printf("initialized KNI protocol successfully\n");
  kni_lm = new CLMBase();
  kni_lm->create((ros::getPackagePath("katana") +
                  "/cfg/stair1_katana.cfg").c_str(), protocol);
  printf("KNI lm base library is up\n");
}

Katana::~Katana()
{
  delete kni_lm;
  delete protocol;
  delete device;
}

bool Katana::calibrate()
{
  kni_lm->calibrate();
  return true;
}

bool Katana::set_motor_power(bool on)
{
  if (on)
    kni_lm->switchRobotOn();
  else
    kni_lm->switchRobotOff();
  return true;
}

bool Katana::allow_crash_limits(bool allow)
{
  if (allow)
    kni_lm->enableCrashLimits();
  else
    kni_lm->disableCrashLimits();
  return true;
}

bool Katana::goto_joint_position_deg(double j1, double j2, double j3,
                                     double j4, double j5)
{
  vector<double> joints;
  joints.push_back(j1);
  joints.push_back(j2);
  joints.push_back(j3);
  joints.push_back(j4);
  joints.push_back(j5);
  kni_lm->moveRobotToDeg(joints.begin(), joints.end(), true);
  // todo: catch exceptions and be smart
  return true;
}

bool Katana::goto_joint_position_deg(int idx, double pos)
{
  kni_lm->movDegrees(idx, pos, true);
  return true;
}

bool Katana::goto_joint_position_rad(double j1, double j2, double j3,
                                     double j4, double j5)
{
	return (goto_joint_position_deg(KNI_MHF::rad2deg(j1), KNI_MHF::rad2deg(j2),
		KNI_MHF::rad2deg(j3), KNI_MHF::rad2deg(j4), KNI_MHF::rad2deg(j5)));
}

bool Katana::goto_joint_position_rad(int idx, double pos)
{
  return (goto_joint_position_deg(idx, KNI_MHF::rad2deg(pos)));
}

bool Katana::gripper_fullstop(bool open_gripper)
{
  try
  {
    if (open_gripper)
      kni_lm->openGripper(true, 10000);
    else
      kni_lm->closeGripper(true, 10000);
    return true;
  }
  catch(MotorTimeoutException &e)
  {
    cout << "Motor timeout." << endl;
    return true;
  }
  catch(MotorCrashException &e)
  {
    cout << "Motor crashed." << endl;
    return false;
  }
}

bool Katana::move_gripper(double fraction_open)
{
  try
  {
    kni_lm->moveGripper(true, 10000, fraction_open);
    return true;
  }
  catch(MotorTimeoutException &e)
  {
    cout << "Motor timeout." << endl;
    return true;
  }
  catch(MotorCrashException &e)
  {
    cout << "Motor crashed." << endl;
    return false;
  }
}

vector<double> Katana::get_joint_positions()
{
  return kni_lm->getJointAngles();
}

vector<int> Katana::get_joint_encoders()
{
  return kni_lm->getRobotEncoders();
}

vector<double> Katana::get_pose()
{
	vector<double> pose(kni_lm->getNumberOfMotors(), 0);
	kni_lm->getCoordinates(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
	return (pose);
}

/*
vector<double> Katana::computeForwardKin(vector<double> joints_rad) 
{
	vector<double> encoders = rad_to_enc(joints_rad);
	vector<double> pose;
	kni_lm->DK(pose, encoders);
}
*/

vector<double> Katana::rad_to_enc(vector<double> joints_rad)
{
	vector<double> encoders;
	vector<TMotInit> motorParams = get_motor_parameters();
	int numberOfMotors = motorParams.size();
	for (int i=0; i<numberOfMotors; i++) {
		encoders.push_back(KNI_MHF::rad2enc(joints_rad[i],
      		motorParams[i].angleOffset, motorParams[i].encodersPerCycle,
      		motorParams[i].encoderOffset, motorParams[i].rotationDirection));
	}
	return (encoders);
}

bool Katana::goto_upright()
{
	printf("going to upright position...\n");
	double goal_state[5] = {3.6, 1.57079633, 1.57079633, 1.57079633, 3.14159625};
  return (goto_joint_position_rad(goal_state[0], goal_state[1], goal_state[2],
          goal_state[3], goal_state[4]));
}

/* Returns joint angle solution in radians. */
bool Katana::ik_calculate(double x, double y, double z, double phi, double theta, double psi, 
      vector<double> &solution)
{
  solution.clear();
  int numberOfMotors = get_number_of_motors();
  vector<int> encoders_solution(numberOfMotors, 0);
  try {
    kni_lm->IKCalculate(x, y, z, phi, theta, psi, encoders_solution.begin());
  } catch(Exception NoSolutionException) {
    cout << "No joint angle solution found." << endl;
    return (false);
  }
  vector<TMotInit> motorParams = get_motor_parameters();
	for (size_t i=0; i<encoders_solution.size(); i++) {
		solution.push_back(KNI_MHF::enc2rad(encoders_solution[i],
      		motorParams[i].angleOffset, motorParams[i].encodersPerCycle,
      		motorParams[i].encoderOffset, motorParams[i].rotationDirection));
  }
  return (true);
}

/*
bool Katana::ik_calculate(double x, double y, double z, double phi, double theta, double psi, 
      vector<double> &solution, vector<int> currentEncoders)
{
  int numberOfMotors = get_number_of_motors();
  vector<int> encoders_solution(numberOfMotors, 0);
  try {
    kni_lm->IKCalculate(x, y, z, phi, theta, psi, encoders_solution.begin(), currentEncoders);
  } catch(Exception NoSolutionException) {
    cout << "No joint angle solution found." << endl;
    return (false);
  }
  vector<TMotInit> motorParams = get_motor_parameters();
	for (int i=0; i<encoders_solution.size(); i++) {
		solution.push_back(KNI_MHF::enc2rad(encoders_solution[i],
      		motorParams[i].angleOffset, motorParams[i].encodersPerCycle,
      		motorParams[i].encoderOffset, motorParams[i].rotationDirection));
  }
  return (true);
}
*/

/* Returns joint angle solution in radians.  If no solution is found for input coordinates and
 * wrist orientation, wrist orientation (theta) is adjusted within specified maximum deviation 
 * (max_wrist_dev) in order to find a solution.
 * Inputs:
 * x, y, z = coordinates of end-effector in Katana frame (in mm)
 * theta = angle of last link (wrist) measured from vertical (0 when wrist end-effector pointed 
 * 		straight up and PI when end-effector pointed straight down) (in radians)
 * psi = angle of wrist -> angle of fifth joint (in radians)
 * max_theta_dev = amount wrist orientation is allowed to deviate from desired value if no solution
 * 		is found for initial orientation (in radians)
 * solution = empty vector to be populated with joint angle solution
 * Note: This function queries the arm to get the current encoder positions to compute a 
 * 		solution. 
*/
bool Katana::ik_joint_solution(double x, double y, double z, double theta_init, double psi, 
		double max_theta_dev, vector<double> &solution)
{
	double angle_inc = 1*PI/180.;
	double phi = atan2((-1.*y),(-1.*x));		// phi is determined by x and y.
  if (phi < 0) {
    phi += 2.*PI;
  }
  cout << "computed phi for given x and y: " << phi << endl;
	double theta = theta_init;
	bool success = false;
  printf("ik_joint_solution arguments:\n");
  printf("x = %f\ny = %f\nz = %f\ntheta = %f\npsi = %f\n", x, y, z, theta, psi);
	
	for (theta = theta_init; theta<=(theta_init+max_theta_dev); theta+=angle_inc) 
	{
    /*if (theta > PI) {
      theta = 2*PI - theta;
    }*/
    if (theta > PI) {
      break;
    }
		cout << "theta = " << theta*180/PI << endl;
  	success = ik_calculate(x,y,z,phi,theta,psi,solution);
		if (success) {
   	 	cout << "Solution: ";
   	 	for (size_t i=0; i<solution.size(); i++) {
   	   cout << solution[i]*180./PI << " ";
   	 	}
   		cout << endl;
			return (true);
  	}
		cout << "Adjusting wrist orientation....";
	}

	for (theta = theta_init; theta>=(theta_init-max_theta_dev); theta-=angle_inc) 
	{
		cout << "theta = " << theta*180/PI << endl;
  	success = ik_calculate(x,y,z,phi,theta,psi,solution);
		if (success) {
   	 	cout << "Solution: ";
   	 	for (size_t i=0; i<solution.size(); i++) {
   	   cout << solution[i]*180./PI << " ";
   	 	}
   		cout << endl;
			return (true);
  	}
		cout << "Adjusting wrist orientation....";
	}

	cout << "No solution found....exiting ik_joint_solution!" << endl;
	return (false);
}

/* Same as above, but currentEncoder values must be input. This function does not need
 * any communication with the arm.
*/
/*
bool Katana::ik_joint_solution(double x, double y, double z, double theta_init, double psi, 
		double max_theta_dev, vector<double> &solution, vector<int> currentEncoders)
{
	double angle_inc = 1*PI/180.;
	double phi = 0;		// phi is determined by x and y....doesn't matter what value it's set to!
	double theta = theta_init;
	bool success = false;
	
	for (theta = theta_init; theta<=(theta_init+max_theta_dev); theta+=angle_inc) 
	{
		cout << "theta = " << theta*180/PI << endl;
  	success = ik_calculate(x,y,z,phi,theta,psi,solution,currentEncoders);
		if (success) {
   	 	cout << "Solution: ";
   	 	for (int i=0; i<solution.size(); i++) {
   	   cout << solution[i]*180./PI << " ";
   	 	}
   		cout << endl;
			return (true);
  	}
		cout << "Adjusting wrist orientation....";
	}

	for (theta = theta_init; theta>=(theta_init-max_theta_dev); theta-=angle_inc) 
	{
		cout << "theta = " << theta*180/PI << endl;
  	success = ik_calculate(x,y,z,phi,theta,psi,solution,currentEncoders);
		if (success) {
   	 	cout << "Solution: ";
   	 	for (int i=0; i<solution.size(); i++) {
   	   cout << solution[i]*180./PI << " ";
   	 	}
   		cout << endl;
			return (true);
  	}
		cout << "Adjusting wrist orientation....";
	}

	cout << "No solution found....exiting ik_joint_solution!" << endl;
	return (false);
}
*/

bool Katana::move_for_camera()
{
	printf("moving out of way for camera...\n");
	//double goal_state1[5] = {5.24, 1.57079633, 1.57079633, 1.57079633, 3.14159625};
//	double goal_state2[5] = {5.24, -.05, 4.02, 2.52, 3.14159625};
	//double goal_state1[5] = {5.24, 1.57079633, 1.57079633, 1.57079633, 3.14159625};
	double goal_state1[5] = {5.24, 1.57079633, 2.8, 1.57079633, 3.14159625};
	//double goal_state2[5] = {5.24, 1.57079633, 4.02, 2.52, 3.14159625};
	double goal_state2[5] = {5.24, 0.8, 4.02, 2.52, 3.14159625};
	double goal_state3[5] = {5.24, 0.0, 4.02, 2.52, 3.14159625};
  if (goto_joint_position_rad(goal_state1[0], goal_state1[1], goal_state1[2],
  goal_state1[3], goal_state1[4])) {
		if (goto_joint_position_rad(goal_state2[0], goal_state2[1], goal_state2[2],
  		goal_state2[3], goal_state2[4])) {
    		return (goto_joint_position_rad(goal_state3[0], goal_state3[1], goal_state3[2],
            goal_state3[3], goal_state3[4]));
		} else return (false); 
  } else return (false);
}

bool Katana::move_back_to_upright()
{
	printf("moving back to upright from out-of-way position...");
	double goal_state1[5] = {5.24, 1.57079633, 1.57079633, 1.57079633, 3.14159625};
	double goal_state2[5] = {3.6, 1.57079633, 1.57079633, 1.57079633, 3.14159625};
  if (goto_joint_position_rad(goal_state1[0], goal_state1[1], goal_state1[2],
  goal_state1[3], goal_state1[4])) {
    return (goto_joint_position_rad(goal_state2[0], goal_state2[1], goal_state2[2],
            goal_state2[3], goal_state2[4]));
  } else return (false);
}

int Katana::get_number_of_motors() 
{
  return(kni_lm->getNumberOfMotors());
}

bool Katana::linear_move(vector<double> dstPose, int waitTimeout)
{
  try
  {
    kni_lm->moveRobotLinearTo(dstPose, true, waitTimeout);
  }
  catch(KNI::NoSolutionException)
  {
    return false;
  }
  return true;
}

/* jointAngles in radians, timeperspline in seconds. */
bool Katana::move_along_trajectory(vector<vector<double> > &jointAngles, double timeperspline) 
{
	int steps = jointAngles.size()-1;

// NEW
	// distance between the end two points
	double v0[3] = {119.087190, 71.691895, 503.301614};
	double v1[3] = {42.730322, 132.270958, 503.301614};
	double distance = sqrt(pow(v0[0]-v1[0], 2.0) + pow(v0[1]-v1[1], 2.0) + pow(v0[2]-v1[2], 2.0));
	// acceleration limits in mm/s^2, hardcoded for now
	double acc = 1500;
	double dec = 1500;
	double vmax = 30; // default value in lmBase.cpp
	// calculate time for whole movement
	// minimum distance to reach vmax
	double borderdistance = pow(vmax, 2.0) / 2.0 * (1 / acc + 1 / dec);
	double totaltime;
	if (distance > borderdistance) {
		totaltime = distance / vmax + vmax / 2.0 * (1 / acc + 1 / dec);
	} else {
		totaltime = sqrt(8 * distance / (acc + dec));
	}
	//double totaltime = kni_lm->totalTime(distance, acc, dec, vmax);
	// calculate number of splines needed
	//double maxtimeperspline = 0.5;
	//int steps = (int) (totaltime / maxtimeperspline) + 1;
	//short timeperspline;
	timeperspline = (short) floor(100*(totaltime/(steps))+1);
// END OF NEW

	// NOTE: Do we want to include gripper motor?
	int numberOfMotors = kni_lm->getNumberOfMotors();
	// Get motor parameters (angleOffset, encoderOffset, etc).
  KNI::kmlFactory *infos = new KNI::kmlFactory();
  string configurationFile = ros::getPackagePath("katana") +
                            "/cfg/stair1_katana.cfg";
	if(!infos->openFile(configurationFile.c_str())) {
		throw ConfigFileOpenException(configurationFile.c_str());
	}
  vector<TMotInit> motorParams = get_motor_parameters();
/*
  for (unsigned int i=0; i<numberOfMotors; i++) {
		printf("angleOffset: %f\n", motorParams[i].angleOffset);
		printf("encodersPerCycle: %d\n", motorParams[i].encodersPerCycle);
		printf("encoderOffset: %d\n", motorParams[i].encoderOffset);
		printf("rotationDir: %d\n", motorParams[i].rotationDirection);
  }
*/
  
	// fill array of timestamps for each joint angle position and check that joint 
		// speeds are within limits
  double timearray[steps+1];
  double time = 0.0;
	double dataarray[steps+1][numberOfMotors];
	vector<int> solution(numberOfMotors, 0), lastsolution(numberOfMotors, 0);

  for (int i=0; i<=steps; i++) {
		timearray[i] = time;
    for (unsigned int j=0; j<numberOfMotors; j++) {
      dataarray[i][j] = KNI_MHF::rad2enc(jointAngles[i][j],
      		motorParams[j].angleOffset, motorParams[j].encodersPerCycle,
      		motorParams[j].encoderOffset, motorParams[j].rotationDirection);
			solution[j] = dataarray[i][j];
		}
		if (time > 0) {
			if (!kni_lm->checkJointSpeeds(lastsolution, solution, timearray[i]-timearray[i-1])) {
				throw JointSpeedException();
			}
		}
		lastsolution.clear();
		lastsolution.assign(solution.begin(), solution.end());
    time += timeperspline;
		printf("%f\n", time);
	}

	//calculate spline
	short parameters[steps][numberOfMotors][7];
	double encoderarray[steps+1];
	double s_time;
	// One motor at a time.
	for (unsigned int i=0; i<numberOfMotors; i++) {
    double arr_p1[steps];
    double arr_p2[steps];
    double arr_p3[steps];
    double arr_p4[steps];

		for (unsigned int j=0; j<=steps; j++) {
			encoderarray[j] = dataarray[j][i];
		}

	  kni_lm->getSplineCoeff(steps, timearray, encoderarray, arr_p1, arr_p2,
			arr_p3, arr_p4);

		for (int j=0; j<steps; j++) {
			// motor number
			parameters[j][i][0] = (short)i;
			// targetencoder
			parameters[j][i][1] = (short)encoderarray[j+1];
			// robot time (in 10ms steps)
			s_time = (timearray[j+1] - timearray[j]) * 100;
			if(j<steps-1)
				parameters[j][i][2] = (short)timeperspline;
			else
				parameters[j][i][2] = (short)s_time;
			// the four spline coefficients
			// the actual position, round
			parameters[j][i][3] = (short)floor(arr_p1[j] + 0.5);
			// shift to be firmware compatible and round
			parameters[j][i][4] = (short)floor(64 * arr_p2[j] / s_time + 0.5);
			parameters[j][i][5] = (short)floor(1024 * arr_p3[j] /
				pow(s_time, 2) + 0.5);
			parameters[j][i][6] = (short)floor(32768 * arr_p4[j] /
				pow(s_time, 3) + 0.5);
		}

  }
  
  // DEBUGGING
  printf("Spline coefficients: \n");
  for (int i=0; i<numberOfMotors; i++) {
    printf("Motor %d: \n",i);
    for (int j=0; j<steps; j++) {
			for (int k=0; k<7; k++) {
      	printf("%d ", parameters[j][i][k]);
			}
			printf("\n");
    }
  }
  // END OF DEBUGGING

	// send spline
	long timeout = -1;	// timeout for a movement in ms.  -1 means endless wait.
	long spline_timeout = (long) parameters[0][0][2] * 10;
	KNI::Timer t(timeout), spline_t(spline_timeout);
	t.Start();
	spline_t.Start();
	//wait for motor
	int wait_timeout = 5000;
	// get master firmware version and revision
	short version, revision;
	kni_lm->GetBase()->getMasterFirmware(&version, &revision);

	bool exactflag = true; // activate the position controller after the movement
	int tolerance = 100; // tolerance for all motor encoders
	for (int i = 0; i<steps; i++) {
		// ignore further steps if timeout elapsed
		if (t.Elapsed()) {
			cout << "Time elapsed before finished.  Exiting." << endl;
			break;
		}
		// wait for motor to finish movement
		kni_lm->waitForMotor(0, 0, tolerance, 2, wait_timeout);
		// send parameters
		for(unsigned int j = 0; j < numberOfMotors; j++) {
			kni_lm->sendSplineToMotor((unsigned short) parameters[i][j][0],
				parameters[i][j][1], parameters[i][j][2],
				parameters[i][j][3], parameters[i][j][4],
				parameters[i][j][5], parameters[i][j][6]);
		}
		// start movement
		kni_lm->startSplineMovement(exactflag);
	}
	
	kni_lm->waitForMotor(0, 0, tolerance, 2, wait_timeout);
	cout << "Done moving." << endl;

  return (true);
}

vector<TMotInit> Katana::get_motor_parameters() 
{
  vector<TMotInit> motorParams;
  unsigned int numberOfMotors = get_number_of_motors();
  for (unsigned int i=0; i<numberOfMotors; i++) {
    TMotInit singleMotor = kni_lm->getMotorParameters(i);
    motorParams.push_back(singleMotor);
  }
  return (motorParams);
}
