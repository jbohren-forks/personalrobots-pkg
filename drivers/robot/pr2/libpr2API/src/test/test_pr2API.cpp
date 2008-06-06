//#include <gazebo/gazebo.h>
//#include <gazebo/GazeboError.hh>

#include <libpr2API/pr2API.h>
#include <math.h>

//using namespace gazebo;
using namespace PR2;

int main()
{
//   PR2::PR2Robot *myPR2 = new PR2::PR2Robot();
   PR2::PR2Robot myPR2;

   myPR2.InitializeRobot();


   myPR2.SetBaseControlMode(PR2_CARTESIAN_CONTROL);
   myPR2.SetBaseCartesianSpeedCmd(0.0,-0.5,1*M_PI/8);
   return 0;
};

