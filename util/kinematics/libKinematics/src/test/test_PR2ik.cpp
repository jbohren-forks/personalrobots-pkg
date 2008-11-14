#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>
#include <sys/time.h>

using namespace kinematics;
using namespace std;


int main()
{
   NEWMAT::Matrix g(4,4);
   g = 0;

   struct timeval t0,t1;

   std::vector<NEWMAT::Matrix> axis;
   std::vector<NEWMAT::Matrix> anchor;
   std::vector<std::string> joint_type;

   NEWMAT::Matrix aj(3,1);
   NEWMAT::Matrix an(3,1);

   joint_type.resize(7);

   // Shoulder pan
   aj << 0 << 0 << 1.0;
   axis.push_back(aj);
   an << 0 << 0 << 0;
   anchor.push_back(an);

   // Shoulder pitch
   aj << 0 << 1 << 0;
   axis.push_back(aj);
   an << 0.1 << 0 << 0;
   anchor.push_back(an);

   // Shoulder roll
   aj << 1 << 0 << 0;
   axis.push_back(aj);
   an << 0.1 << 0 << 0;
   anchor.push_back(an);

   // Elbow flex
   aj << 0 << 1 << 0;
   axis.push_back(aj);
   an << 0.5 << 0 << 0;
   anchor.push_back(an);

   // Forearm roll
   aj << 1 << 0 << 0;
   axis.push_back(aj);
   an << 0.5 << 0 << 0;
   anchor.push_back(an);

   // Wrist flex
   aj << 0 << 1 << 0;
   axis.push_back(aj);
   an << 0.82025 << 0 << 0;
   anchor.push_back(an);

   // Gripper roll
   aj << 1 << 0 << 0;
   axis.push_back(aj);
   an << 0.82025 << 0 << 0;
   anchor.push_back(an);

   for(int i=0; i < 7; i++)
     joint_type[i] = std::string("ROTARY");

   arm7DOF myArm(anchor,axis,joint_type);

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

   gettimeofday(&t0,NULL);

   myArm.ComputeIK(g,0.1);

   gettimeofday(&t1,NULL);

   myArm.ComputeIK(g,0.2);

   double time_taken = (t1.tv_sec*1000000+t1.tv_usec - (t0.tv_sec*1000000+t0.tv_usec))/1000000.;

   cout << "Time taken " << time_taken << endl;

   for(int i=0; i < (int) myArm.solution_ik_.size(); i++)
   {
     for(int j=0; j < 7; j++)
     {
       cout << myArm.solution_ik_[i][j] << " " ;
     }
     cout << endl;
   }
//   PrintMatrix(theta,"Answer");

#ifdef DEBUG
   cout << "Main::End-effector Pose:" << endl << g << endl ;
   cout << "Main::End-effector Velocity:" << endl << eS;
#endif
}

