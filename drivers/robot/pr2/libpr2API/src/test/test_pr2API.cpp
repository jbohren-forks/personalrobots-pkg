#include <libKinematics/ik.h>
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace kinematics;
using namespace PR2;
using namespace std;

int main()
{
   // Test for pr2API
   PR2::PR2Robot myPR2; // Setup the robot 
   myPR2.InitializeRobot(); // Initialize the robot (sets up kinematics, etc.)

   // set random numbers to base cartesian control
//   myPR2.SetBaseControlMode(PR2_CARTESIAN_CONTROL);
//   myPR2.SetBaseCartesianSpeedCmd(0.0,-0.5,1*M_PI/8);

   // test pitch the hokuyo
   myPR2.hw.SetJointServoCmd(PR2::HEAD_LASER_PITCH, -M_PI/8.0, 0.0);

   // test kinematics library through pr2API
   PR2Arm myArm;

   // create a end-effector position and orientation for inverse kinematics test
   NEWMAT::Matrix g(4,4);
   g = 0;

   // Joint angles (radians) and speeds for testing 
   double angles[7] = {0,0,0,0,0,0,0};
   //double speeds[7] = {0,0,0,0,0,0,0};

   //int iii;
/*
   angles[0] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[0] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[0] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


   angles[1] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[1] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[1] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;



   angles[2] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[2] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[2] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


   angles[3] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[3] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[3] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;



   angles[4] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[4] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[4] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


  angles[5] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[5] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[5] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


  angles[6] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[6] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[6] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;





















   angles[0] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[0] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[0] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


   angles[1] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[1] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[1] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;



   angles[2] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[2] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[2] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


   angles[3] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[3] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[3] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;



   angles[4] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[4] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[4] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


  angles[5] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[5] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[5] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


  angles[6] = 0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);    // send command to robot
   sleep(4);

   angles[6] = -0.5;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   angles[6] = -0;
   myPR2.SetArmJointPosition(PR2::PR2_LEFT_ARM,angles,speeds);   // send command to robot
   sleep(4);

   cout << "Next joint?" << endl;
   cin >> iii;


*/



























   angles[0] = 0.1; // shoulder pan angle
   angles[1] = -1;  // shoulder lift angle
   angles[2] = 0.3; // upperarm roll angle
   angles[3] = 0.3; // elbow pitch angle
   angles[4] = 0.2; // forearm roll angle
   angles[5] = 0.5; // wrist pitch angle
   angles[6] = 0.0; // wrist roll


   g = myPR2.ComputeArmForwardKinematics(PR2::PR2_RIGHT_ARM,angles);
   cout << "Main::End-effector Pose:" << endl << g << endl ;

   // send command to robot
   myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,g);

   NEWMAT::Matrix theta(8,8);
   theta = 0;
   theta = myPR2.ComputeArmInverseKinematics(PR2::PR2_RIGHT_ARM,g);
   PrintMatrix(theta,"exact solution for pos/orien of end effector");

   NEWMAT::Matrix g_wrist(4,4);
   double x;
   double y;
   double z; 
   double roll;
   double pitch; 
   double yaw;

   myPR2.hw.GetWristPoseGroundTruth(PR2::PR2_RIGHT_ARM,&x,&y,&z,&roll,&pitch,&yaw);
   g_wrist = myPR2.hw.GetWristPoseGroundTruth(PR2::PR2_RIGHT_ARM);

   cout << "Right wrist::" << endl;
   cout << "pos:: (" << x << "," << y << "," << z << ")" << endl;
   cout << "rot:: (" << roll << "," << pitch << "," << yaw << ")" << endl;
   PrintMatrix(g_wrist,"Wrist ground truth");

   double jointPosition[7];
   double jointSpeed[7];

   myPR2.GetArmJointPositionActual(PR2::PR2_RIGHT_ARM,jointPosition,jointSpeed);

   for(int ii=0; ii<7; ii++)
      cout << "Joint ii::" << ii << ":: " << jointPosition[ii] << endl;

/*   NEWMAT::Matrix pose = myArm.ComputeFK(angles);
   g = pose;
   // offset from the base of the arm,
   // subtracted for PR2 to cancel out effect of offset
   // watchout whether left of right arm is used
   g(1,4) = g(1,4) - PR2::SPINE_RIGHT_ARM_OFFSET.x;
   g(2,4) = g(2,4) - PR2::SPINE_RIGHT_ARM_OFFSET.y;
   g(3,4) = g(3,4) - PR2::SPINE_RIGHT_ARM_OFFSET.z;
 
   // compute analytical solution
   NEWMAT::Matrix theta(8,8);
   theta = 0;
   theta = myArm.ComputeIK(g,0.1);
   PrintMatrix(theta,"exact solution for pos/orien of end effector");

   cout << "Main::End-effector Pose:" << endl << g << endl ;
   // remove offset to check results of inverse kinematics
   g(1,4) = g(1,4) + PR2::SPINE_RIGHT_ARM_OFFSET.x;
   g(2,4) = g(2,4) + PR2::SPINE_RIGHT_ARM_OFFSET.y;
   g(3,4) = g(3,4) + PR2::SPINE_RIGHT_ARM_OFFSET.z;

   g(1,4) = g(1,4) + PR2::SPINE_RIGHT_ARM_OFFSET.x;
   g(2,4) = g(2,4) + PR2::SPINE_RIGHT_ARM_OFFSET.y;
   g(3,4) = g(3,4) + PR2::SPINE_RIGHT_ARM_OFFSET.z;

   // send command to robot
   myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,g);


*/
   return 0;
};
