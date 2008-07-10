
#include "libKDL/kdl_kinematics.h"

#include <sys/time.h>
#include <unistd.h>


using namespace KDL;
using namespace PR2;
using namespace std;



int main( int argc, char** argv )
{
	PR2_kinematics pr2_kin;
	struct timeval t0, t1;

	JntArray pr2_config = JntArray(pr2_kin.nJnts);
	pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
	pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;
	cout<<"Config of the arm:"<<pr2_config<<endl;

	Frame f;
	if (pr2_kin.FK(pr2_config,f))
		cout<<"End effector transformation:"<<f<<endl;
	else
		cout<<"Could not compute Fwd Kin."<<endl;

	JntArray q_init = JntArray(pr2_kin.nJnts);
	q_init(0) = 0.1, q_init(1) = 0.0, q_init(2) = 0.0, q_init(3) = 0.0;
	q_init(4) = 0.0, q_init(5) = 0.0, q_init(6) = 0.0;

	JntArray q_out = JntArray(pr2_kin.nJnts);
	gettimeofday(&t0,NULL);
	if (pr2_kin.IK(q_init, f, q_out) == true)
	{
		gettimeofday(&t1, NULL);
		cout<<"IK result:"<<q_out<<endl;
		double time_taken = (t1.tv_sec*1000000+t1.tv_usec - (t0.tv_sec*1000000+t0.tv_usec))/1000.;
	  printf("Time taken: %f ms\n", time_taken);
	}
	else
		cout<<"Could not compute Inv Kin."<<endl;

	//------ checking that IK returned a valid soln -----
	Frame f_ik;
	if (pr2_kin.FK(q_out,f_ik))
		cout<<"End effector after IK:"<<f_ik<<endl;
	else
		cout<<"Could not compute Fwd Kin. (After IK)"<<endl;

}



