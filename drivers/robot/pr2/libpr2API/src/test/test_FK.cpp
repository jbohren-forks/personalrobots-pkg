#include <libKinematics/ik.h>
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace kinematics;
using namespace PR2;
using namespace std;

int main()
{
   cout << "Starting up" << endl;

   PR2::PR2State rS;
   cout << "Starting up 2" << endl;
   PR2::PR2Robot myPR2; // Setup the robot 
   cout << "Starting up 3" << endl;

   NEWMAT::Matrix g(4,4);

//   myPR2.InitializeRobot(); // Initialize the robot (sets up kinematics, etc.)


   rS.pos.x = 0;
   rS.pos.y = 0;
   rS.pos.z = 0;
   rS.rightArmJntAngles[0] = 0.1; // shoulder pan angle
   rS.rightArmJntAngles[1] = -1;  // shoulder lift angle
   rS.rightArmJntAngles[2] = 0.3; // upperarm roll angle
   rS.rightArmJntAngles[3] = 0.3; // elbow pitch angle
   rS.rightArmJntAngles[4] = 0.2; // forearm roll angle
   rS.rightArmJntAngles[5] = 0.5; // wrist pitch angle
   rS.rightArmJntAngles[6] = 0.0; // wrist roll


   g = myPR2.ComputeBodyPose(PR2::ARM_L_PAN,rS);

   cout << g;

   return 0;
};
