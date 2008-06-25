#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>
#include <signal.h>

// roscpp
#include <ros/node.h>

// for frame transforms
#include <rosTF/rosTF.h>

using namespace PR2;
using namespace std;

static int notDone = 1;

void finalize(int dummy)
{
   notDone = 0;
}

int main(int argc, char** argv)
{

 
   ros::init(argc,argv);

   ros::node myNode("testFK");
   rosTFServer tfServer(myNode);

   PR2::PR2State rS;
   PR2::PR2Robot myPR2; // Setup the robot 

   NEWMAT::Matrix g(4,4);

  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

   double x;
   double y;
   double z; 
   double roll;
   double pitch; 
   double yaw;
   int ii;

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

   while(notDone)
   {
      // Publish the base
      for(ii = (int) PR2::CASTER_FL_STEER; ii <= (int) PR2::CASTER_RR_DRIVE_R; ii++) 
      {
         myPR2.ComputeBodyPose((PR2::PR2_JOINT_ID)ii,rS,&x,&y,&z,&roll,&pitch,&yaw);
         tfServer.sendEuler((unsigned int) ii,(unsigned int) PR2::PR2_WORLD,x,y,z,yaw,pitch,roll,0,0);
      }

      // Publish the arms
      for(ii = (int) PR2::ARM_L_PAN; ii <= (int) PR2::ARM_R_WRIST_ROLL; ii++) 
      {
         myPR2.ComputeBodyPose((PR2::PR2_JOINT_ID)ii,rS,&x,&y,&z,&roll,&pitch,&yaw);
         tfServer.sendEuler((unsigned int) ii,(unsigned int) PR2::PR2_WORLD,x,y,z,yaw,pitch,roll,0,0);
      }

      // Publish the head
      for(ii = (int) PR2::HEAD_YAW; ii <= (int) PR2::HEAD_PITCH; ii++) 
      {
         myPR2.ComputeBodyPose((PR2::PR2_JOINT_ID)ii,rS,&x,&y,&z,&roll,&pitch,&yaw);
         tfServer.sendEuler((unsigned int) ii,(unsigned int) PR2::PR2_WORLD,x,y,z,yaw,pitch,roll,0,0);
      }
    usleep(1000000);
    cout << "Publishing:: " << endl;

   }

   ros::fini();

   return 0;
};


/*example pseudo code for recieving
   {

   rosTF::rosTFClient tfClient(myNode);
   

   TFPose aPose ("WristFrameID", 0,0,0,0,0,0, time);


   TFPose inBaseFrame = tfClient.transformPose("baseFrameID", aPose);
   }
*/
