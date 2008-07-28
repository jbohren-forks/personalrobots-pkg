
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

using namespace KDL;
using namespace PR2;
using namespace std;


int main()
{
	// test pr2API
	PR2::PR2Robot myPR2;

	myPR2.InitializeRobot();

	// set random numbers to base cartesian control
	myPR2.SetBaseControlMode(PR2_CARTESIAN_CONTROL);

	//------------
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


	JntArray pr2_config = JntArray(myPR2.pr2_kin.nJnts);
	pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
	pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;
	cout<<"Config of the arm:"<<pr2_config<<endl;

	Frame f;
	if (myPR2.pr2_kin.FK(pr2_config,f))
		cout<<"End effector transformation:"<<f<<endl;
	else
		cout<<"Could not compute Fwd Kin."<<endl;

	// send command to robot
	(*myPR2.pr2_kin.q_IK_guess)(0) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(1) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(2) = 0.0;
	(*myPR2.pr2_kin.q_IK_guess)(3) = 0.0,	(*myPR2.pr2_kin.q_IK_guess)(4) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(5) = 0.0;
	(*myPR2.pr2_kin.q_IK_guess)(6) = 0.0;
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f);


	myPR2.GetBasePositionActual(&x, &y, &z, &roll, &pitch, &yaw);
	printf("x: %f, y: %f, roll: %f pitch: %f yaw: %f\n",x,y,roll,pitch,yaw);

	return 0;
}


