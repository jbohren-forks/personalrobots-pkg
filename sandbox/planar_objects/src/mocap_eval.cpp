/*
 * box_tracker.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "mocap_eval.h"
#include "ros/node.h"

#define PRINTVEC(a) a.x()<<" "<<a.y()<<" "<<a.z()<<" "
#define SQR(a) ((a)*(a))
#define sqr(a) ((a)*(a))

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
	ros::init(argc, argv, "mocap_eval");

	planar_objects::MocapEval node;
	ros::spin();

	return 0;
}

namespace planar_objects {

// Constructor
MocapEval::MocapEval() {
	newLines = 0;
	oldLines = 0;

	// subscribe to topics
	mocap_sub = nh.subscribe("/phase_space_snapshot", 1,
			&MocapEval::mocapCallback, this);
	observations_sub = nh.subscribe("box_detector/observations", 1,
			&MocapEval::observationsCallback, this);

	// advertise topics
	visualization_pub = nh.advertise<visualization_msgs::Marker> (
			"~visualization_marker", 100);

	cloud_pub = nh.advertise<sensor_msgs::PointCloud> (
			"~cloud", 100);

	 	notifier = new tf::MessageNotifier<BoxObservations>(
	                   tf,
	                   boost::bind(&MocapEval::callback,this,_1),
	                   "box_detector/observations", "base_link", 50);
	notifier->setTolerance(ros::Duration(0.03));

}

void MocapEval::mocapCallback(const mocap_msgs::MocapSnapshotConstPtr& msg) {
	this->mocap_msg = msg;

	mocap_markers.clear();
	for (size_t i = 0; i < mocap_msg->markers.size(); i++) {
		int id = mocap_msg->markers[i].id;
		double x = mocap_msg->markers[i].location.x;
		double y = mocap_msg->markers[i].location.y;
		double z = mocap_msg->markers[i].location.z;
		mocap_markers[id] = btVector3(x, y, z);
	}

	sendPointCloud();

	for (size_t i = 0; i < mocap_msg->bodies.size(); i++) {
		int id = mocap_msg->bodies[i].id;
		double x = mocap_msg->bodies[i].pose.translation.x;
		double y = mocap_msg->bodies[i].pose.translation.y;
		double z = mocap_msg->bodies[i].pose.translation.z;
		double rx = mocap_msg->bodies[i].pose.rotation.x;
		double ry = mocap_msg->bodies[i].pose.rotation.y;
		double rz = mocap_msg->bodies[i].pose.rotation.z;
		double rw = mocap_msg->bodies[i].pose.rotation.w;
		mocap_bodies[id] = btTransform(btQuaternion(rx, ry, rz, rw), btVector3(
				x, y, z));
	}

	if( mocap_markers.find(4097) != mocap_markers.end() &&
		mocap_markers.find(4100) != mocap_markers.end() &&
		mocap_markers.find(4099) != mocap_markers.end() ) {

		mocap_obs.w = (mocap_markers[4097] - mocap_markers[4099]).length();
		mocap_obs.h = (mocap_markers[4097] - mocap_markers[4100]).length();
	}

	if( mocap_bodies.find(1) != mocap_bodies.end() ) {
		btTransform t(btQuaternion( 0.0200151,0.718508,0.695192,0.00731243),btVector3(0.468977,1.24,-0.570441));
		mocap_obs.tf = t * mocap_bodies[1];
//		mocap_obs.tf = mocap_bodies[1];

	}
}

void MocapEval::sendPointCloud() {
	if(observations_msg.get() == NULL) return;
	sensor_msgs::PointCloud points;
	points.header = observations_msg->header;
	points.set_points_size(mocap_markers.size());
	points.set_channels_size(2);
	points.channels[0].name="rgb";
	points.channels[1].name="id";
	geometry_msgs::Point32 p;
	points.channels[0].set_values_size(mocap_markers.size());
	points.channels[1].set_values_size(mocap_markers.size());

	std::map<int,btVector3>::iterator it = mocap_markers.begin();
	int j=0;
	for (; it !=mocap_markers.end(); it++) {
		points.points[j].x = it->second.x();
		points.points[j].y = it->second.y();
		points.points[j].z = it->second.z();
		int rgb=0xffffff;
		points.channels[0].values[j]=*(float*) &rgb;
		points.channels[1].values[j]=it->first;
		j++;
	}

	cloud_pub.publish(points);
}

btTransform MocapEval::transformToBaseLink( std::string fromFrame, std::string toFrame, btTransform pose ) {
	btTransform poseOut;

	geometry_msgs::PoseStamped stamped_in;
	stamped_in.header = observations_msg->header;
//	stamped_in.header.stamp += ros::Duration(0.026);
	stamped_in.header.frame_id = fromFrame;
	stamped_in.pose.orientation.x = pose.getRotation().x();
	stamped_in.pose.orientation.y = pose.getRotation().y();
	stamped_in.pose.orientation.z = pose.getRotation().z();
	stamped_in.pose.orientation.w = pose.getRotation().w();
	stamped_in.pose.position.x = pose.getOrigin().x();
	stamped_in.pose.position.y = pose.getOrigin().y();
	stamped_in.pose.position.z = pose.getOrigin().z();

	geometry_msgs::PoseStamped stamped_out;
	try {
		tf.transformPose(toFrame,stamped_in,stamped_out);
	} catch(tf::TransformException& ex)
	{
		cout << "could not transform "<< ex.what() << endl;
		return(pose);
	}

	poseOut = btTransform(
			btQuaternion(
					stamped_out.pose.orientation.x,
					stamped_out.pose.orientation.y,
					stamped_out.pose.orientation.z,
					stamped_out.pose.orientation.w
					),
			btVector3(
					stamped_out.pose.position.x,
					stamped_out.pose.position.y,
					stamped_out.pose.position.z));
	return(poseOut);


}

void MocapEval::observationsCallback(
		const BoxObservations::ConstPtr& new_observations) {
}

void MocapEval::callback(
  const tf::MessageNotifier<BoxObservations>::MessagePtr& new_observations) {

	observations_msg = new_observations;
	header = observations_msg->header;
	header.frame_id="base_link";
	setVisualization(&visualization_pub, NULL, header);
	ROS_INFO("BoxTracker::syncCallback(), received %d observations, %d markers, %d bodies",observations_msg->get_obs_size(),mocap_markers.size(),mocap_bodies.size());
	observations.clear();

//	cout << "phasespace "<<mocap_obs.toString()<<endl;

	for (size_t i = 0; i < observations_msg->obs.size(); i++) {
		btBoxObservation obs = btBoxObservation(observations_msg->obs[i],
				observations_msg->header.stamp);

		obs.tf = transformToBaseLink(  observations_msg->header.frame_id,"base_link",obs.tf );


		std::vector<btBoxObservation> ambiguity;
		ambiguity.push_back( obs.getAmbiguity(0) );
		ambiguity.push_back( obs.getAmbiguity(1) );
		ambiguity.push_back( obs.getAmbiguity(2) );
		ambiguity.push_back( obs.getAmbiguity(3) );

		// has smallest or second-smallest x axis
		std::map<double, btBoxObservation> sortX;
		for(size_t i=0;i<ambiguity.size();i++)
			sortX[ ambiguity[i].tf.getOrigin().x() ] =  ambiguity[i];

		// of the remaining two, has smallest z axis
		std::map<double, btBoxObservation> sortZ;
		sortZ[ sortX.begin()->second.tf.getOrigin().z() ] = sortX.begin()->second;
		sortX.erase(sortX.begin());
		sortZ[ sortX.begin()->second.tf.getOrigin().z() ] = sortX.begin()->second;

		obs = sortZ.begin()->second;
		if(obs.w > mocap_obs.w*1.5 || obs.w < mocap_obs.w*0.5 ) continue;
		if(obs.h > mocap_obs.h*1.5 || obs.h < mocap_obs.h*0.5 ) continue;

		observations.push_back( obs );
	}

	for (size_t i = 0; i < observations.size(); i++) {
		btTransform uncalibrated = observations[i].tf.inverse() * mocap_obs.tf;

//		cout << "uncalibrated: "<<PRINTTF(uncalibrated) << endl;
//		btTransform t(btQuaternion( 0.0200151,0.718508,0.695192,0.00731243),btVector3(0.468977,1.24,-0.570441));
//		cout << PRINTTF( observations[i].tf * mocap_obs.tf.inverse() ) << endl;
//		btTransform calibrated = observations[i].tf.inverse() * (t*mocap_obs.tf);
//		cout << "calibrated: "<<PRINTTF(calibrated) << endl;
//		observations[i].tf = observations[i].tf;
	}

	visualizeObservations();
	removeOldLines();
}

void MocapEval::removeOldLines() {
	LineVector lines;
	for (int l = newLines; l < oldLines; l++) {
		visualizeLines(lines, l, 0, 0, 0);
	}
	oldLines = newLines;
	newLines = 0;
}

void MocapEval::visualizeObservations() {
	for (size_t i = 0; i < observations.size(); i++) {
		//    cout << "drawing line "<<newLines << endl;
		visualizeLines(observations[i].visualize(), newLines++, HSV_to_RGB(0.5,
				1.0, 1.0));
	}
	visualizeLines(mocap_obs.visualize(), newLines++, HSV_to_RGB(0.0,
			1.0, 1.0));
}

}
