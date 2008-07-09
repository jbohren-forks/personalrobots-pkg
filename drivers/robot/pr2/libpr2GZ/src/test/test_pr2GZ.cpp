#include <libpr2GZ/pr2GZ.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace PR2;
using namespace std;

int main()
{
   // Test for pr2GZ
   PR2::PR2GZ myPR2; // Setup the robot 
   myPR2.UpdateGZ(); // Initialize the robot (sets up kinematics, etc.)

   return 0;
};
