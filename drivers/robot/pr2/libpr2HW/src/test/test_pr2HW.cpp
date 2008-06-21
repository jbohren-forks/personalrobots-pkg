#include <libpr2HW/pr2HW.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace PR2;
using namespace std;

int main()
{
   // Test for pr2HW
   PR2::PR2HW myPR2; // Setup the robot 
   myPR2.UpdateHW(); // Initialize the robot (sets up kinematics, etc.)

   return 0;
};
