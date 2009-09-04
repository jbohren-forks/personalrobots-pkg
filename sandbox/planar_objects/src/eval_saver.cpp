/*
 * box_tracker.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "eval_saver.h"
#include "ros/node.h"

#include <fstream>

#define PRINTVEC(a) a.x()<<" "<<a.y()<<" "<<a.z()<<" "
#define SQR(a) ((a)*(a))
#define sqr(a) ((a)*(a))
#define CVWINDOW(a) cvNamedWindow(a,CV_WINDOW_AUTOSIZE); cvMoveWindow(a,(cvWindows /3 )* 500,(cvWindows%3)*500); cvWindows++;

#define PRINTTF(tf) \
	"btTransform(btQuaternion(" <<\
	" "<<(tf).getRotation().x()<< \
	","<<(tf).getRotation().y()<< \
	","<<(tf).getRotation().z()<< \
	","<<(tf).getRotation().w() << \
	"),btVector3("<<(tf).getOrigin().x()<< \
	","<<(tf).getOrigin().y()<< \
	","<<(tf).getOrigin().z()<< "))"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "eval_saver");

	planar_objects::EvalSaver node;
	ros::spin();

	return 0;
}

namespace planar_objects {

// Constructor
EvalSaver::EvalSaver() {
	// subscribe to topics
	eval_sub = nh.subscribe("mocap_eval/eval", 0,&EvalSaver::evalCallback, this);
}

void EvalSaver::evalCallback(const MocapEvalObservations::ConstPtr& msg) {
	eval_msg = msg;

	ofstream obs_file ( "eval-obs.txt", ios::app );
	ofstream rec_file ( "eval-rec.txt", ios::app );

	for (size_t i = 0; i < msg->obs.size(); i++) {
		stringstream s;
		s <<msg->dist_to_camera<<" "<<(msg->angle_to_camera/M_PI*180.)<<" "<<
			msg->obs[i].err_trans<<" "<<(msg->obs[i].err_rot/M_PI*180.0)<<" " <<
			msg->obs[i].w<<" "<<msg->obs[i].h<<" "<<
			msg->obs[i].plane_id<<" "<<
			endl;

		obs_file << s.str();
		cout <<s.str();
	}

	stringstream s;
	s <<msg->dist_to_camera<<" "<<(msg->angle_to_camera/M_PI*180.)<<" "<<msg->obs.size()<<endl;

	rec_file << s.str();
	cout <<s.str();
}

}
