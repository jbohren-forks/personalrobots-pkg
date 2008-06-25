#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace PR2;
using namespace std;

int main()
{

   PR2::PR2State rS;
   PR2::PR2Robot myPR2; // Setup the robot 

   NEWMAT::Matrix g(4,4);

   double x;
   double y;
   double z; 
   double roll;
   double pitch; 
   double yaw;


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
   rS.spineExtension = 0.0;

   myPR2.ComputeBodyPose(PR2::ARM_R_PAN,rS,&x,&y,&z,&roll,&pitch,&yaw);
   cout << "pos:: (" << x << "," << y << "," << z << ")" << endl;
   cout << "rot:: (" << roll << "," << pitch << "," << yaw << ")" << endl;


   g = myPR2.ComputeBodyPose(PR2::ARM_R_WRIST_ROLL,rS);
   cout << g;

   return 0;
};
