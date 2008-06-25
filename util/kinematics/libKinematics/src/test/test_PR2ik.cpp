#include <libKinematics/kinematics.h>
#include <libKinematics/ik.h>

using namespace kinematics;
using namespace PR2;
using namespace std;


int main()
{
   NEWMAT::Matrix g(4,4);
   g = 0;

   PR2Arm myArm;

/* Joint angles and speeds for testing */
   double angles[7] = {0,0,0,0,0,0,0};
   double speeds[7] = {0,0,0,0,0,0,0};


/* Return test values */
   NEWMAT::Matrix g0 = myArm.ComputeFK(angles);
   NEWMAT::Matrix eS = myArm.ComputeEndEffectorVelocity(angles,speeds);

   angles[0] = 0.1;
   angles[1] = -1;
   angles[2] = 0.3;
   angles[3] = 0.3;
   angles[4] = 0.2;
   angles[5] = 0.5;

   NEWMAT::Matrix pose = myArm.ComputeFK(angles);
   g = pose;

   NEWMAT::Matrix theta(7,1);
   theta = 0;
   theta = myArm.ComputeIK(g,0.1);

   PrintMatrix(theta,"Answer");
#ifdef DEBUG
   cout << "Main::End-effector Pose:" << endl << g << endl ;
   cout << "Main::End-effector Velocity:" << endl << eS;
#endif
}

