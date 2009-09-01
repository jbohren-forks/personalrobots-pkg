/*
 * articulation_models.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#include "articulation_models.h"
#include "Eigen/Core"
#include <Eigen/SVD>
#include "opencv/cv.h"

#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_circle.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

#define PRINTVEC(a) a.x()<<" "<<a.y()<<" "<<a.z()<<" "
#define SQR(a) ((a)*(a))
#define sqr(a) ((a)*(a))

namespace planar_objects {

ManifoldModel::ManifoldModel(btBoxTrack* track):track(track) {
	err_rot = DBL_MAX;
	err_trans = DBL_MAX;
}

void ManifoldModel::findParameters(ros::Publisher* pub) {
	w = 0;
	h = 0;
	if (track->obs_history.size() < 2)
		return;

	double sum_w = 0, sum_h = 0;
	for (size_t j = 0; j < track->obs_history.size(); j++) {
		sum_w += track->obs_history[j].w;
		sum_h += track->obs_history[j].h;
	}
	w = sum_w / track->obs_history.size();
	h = sum_h / track->obs_history.size();
}

double ManifoldModel::getTranslationalError() {
	return (err_trans);
}

double ManifoldModel::getRotationalError() {
	return (err_rot);
}

double ManifoldModel::getLoglikelihood(double sigma_trans,double sigma_rot) {
    return - ( sqr(err_rot)/sqr(sigma_rot) + sqr(err_trans)/sqr(sigma_trans) );
}

bool ManifoldModel::isValid() {
	return (true);
}

void ManifoldModel::getError(double &err_trans, double &err_rot) {
	err_trans = this->err_trans;
	err_rot = this->err_rot;
}

void ManifoldModel::computeError() {
	err_rot = DBL_MAX;
	err_trans = DBL_MAX;
	if (track->obs_history.size() < 2 || !isValid())
		return;

	double sum_trans = 0, sum_rot = 0;
	for (size_t j = 0; j < track->obs_history.size(); j++) {
		btTransform pred = getPrediction(getConfiguration(
				track->obs_history[j].tf));
		btTransform error = track->obs_history[j].tf.inverseTimes(pred);
		sum_trans += error.getOrigin().length();
		sum_rot += error.getRotation().getAngle();
	}
	err_trans = sum_trans / track->obs_history.size();
	err_rot = sum_rot / track->obs_history.size();

	if (!isnormal(err_trans))
		err_trans = DBL_MAX;
	if (!isnormal(err_rot))
		err_rot = DBL_MAX;
}

std::vector<std::pair<double, double> > ManifoldModel::getRange() {
	std::vector<std::pair<double, double> > range;
	range.resize(getDOFs());
	if (track->obs_history.size() == 0)
		return range;

	std::vector<double> q = getConfiguration(track->obs_history[0].tf);
	for (size_t i = 0; i < getDOFs(); i++) {
		range[i].first = q[i];
		range[i].second = q[i];
	}
	for (size_t j = 1; j < track->obs_history.size(); j++) {
		std::vector<double> q = getConfiguration(track->obs_history[j].tf);
		for (size_t i = 0; i < getDOFs(); i++) {
			range[i].first = MIN(range[i].first, q[i]);
			range[i].second = MAX(range[i].second, q[i]);
		}
	}

	return (range);
}

btBoxObservation ManifoldModel::getPredictedObservation(std::vector<double> q) {
	btBoxObservation result;
	result.w = w;
	result.h = h;
	result.precision = 1.0;
	result.recall = 1.0;
	result.tf = getPrediction(q);
	return (result);
}

PrismaticModel::PrismaticModel(btBoxTrack* track) :
	ManifoldModel(track) {
}

void PrismaticModel::findParameters(ros::Publisher* pub) {
	ManifoldModel::findParameters();

	if (track->obs_history.size() == 0) {
		ROS_ERROR("PrismaticModel::findParameters on empty history!!");
		return;
	}
	// find center
	btQuaternion o = track->obs_history[0].tf.getRotation();
	btVector3 t = track->obs_history[0].tf.getOrigin();

	for (size_t j = 1; j < track->obs_history.size(); j++) {
		o = o.slerp(track->obs_history[j].tf.getRotation(), 1 / (j + 1.0));
		t = t.lerp(track->obs_history[j].tf.getOrigin(), 1 / (j + 1.0));
	}

	center = btTransform(o, t);

	if (track->obs_history.size() < 6) {
		return;
	}

	// find line direction
	MatrixXf X_trans(track->obs_history.size(), 3);
	for (size_t j = 0; j < track->obs_history.size(); j++) {
		X_trans(j, 0)
				= center.inverseTimes(track->obs_history[j].tf).getOrigin().x();
		X_trans(j, 1)
				= center.inverseTimes(track->obs_history[j].tf).getOrigin().y();
		X_trans(j, 2)
				= center.inverseTimes(track->obs_history[j].tf).getOrigin().z();
	}
	//  std::cout << "Matrix X is:" << std::endl << std::endl << X_trans << std::endl << std::endl;

	VectorXf b(track->obs_history.size());
	VectorXf x(3);
	SVD<MatrixXf> svdOfA(X_trans);
	svdOfA.solve(b, &x);

	const Eigen::MatrixXf U = svdOfA.matrixU();
	const Eigen::MatrixXf V = svdOfA.matrixV();
	const Eigen::VectorXf S = svdOfA.singularValues();

	DiagonalMatrix<Eigen::VectorXf> S2(S);
	//
	//  std::cout << "Matrix U is:" << std::endl << std::endl << U << std::endl << std::endl;
	//  std::cout << "Matrix V is:" << std::endl << std::endl << V << std::endl << std::endl;
	//  std::cout << "Vector S is:" << std::endl << std::endl << S << std::endl << std::endl;
	//  std::cout << "Matrix S2 is:" << std::endl << std::endl << S2 << std::endl << std::endl;
	//  std::cout << "Vector x is:" << std::endl << std::endl << x << std::endl << std::endl;
	//  std::cout << "Vector b is:" << std::endl << std::endl << b << std::endl << std::endl;

	Eigen::MatrixXf Y_trans = V * S2;
	//  std::cout << "Matrix Y_trans is:" << std::endl << std::endl << Y_trans << std::endl << std::endl;

	direction = btVector3(V(0, 0), V(1, 0), V(2, 0)).normalize();

	//  cout <<"estimating q's:"<<endl;
	//  for(size_t j=0;j<track->obs_history.size();j++) {
	//    cout << "j="<<j<<" q[0]="<<getConfiguration(track->obs_history[j].tf)[0]<<endl;
	//  }
	//
	//  cout <<"translational error:"<<getTranslationalError()<<endl;
	//  cout <<"rotational error:"<<getRotationalError()<<endl;

	cout << "current range: " << getRange()[0].first << " .. "
			<< getRange()[0].second << endl;
}

size_t PrismaticModel::getDOFs() {
	return (1);
}

std::vector<double> PrismaticModel::getConfiguration(btTransform transform) {
	std::vector<double> q;
	q.resize(1);
	q[0] = direction.dot(center.inverseTimes(transform).getOrigin());
	return (q);
}

btTransform PrismaticModel::getPrediction(std::vector<double> q) {
	if (q.size() != 1)
		ROS_ERROR("PrismaticModel::getPrediction must have 1 DOF");
	return (center * btTransform(btQuaternion::getIdentity(), direction * q[0]));
}

bool PrismaticModel::isValid() {
	if (track->obs_history.size() < 6)
		return (false);
	if (!isnormal(center.getOrigin().x()))
		return (false);
	if (!isnormal(center.getOrigin().y()))
		return (false);
	if (!isnormal(center.getOrigin().z()))
		return (false);
	if (!isnormal(center.getRotation().x()))
		return (false);
	if (!isnormal(center.getRotation().y()))
		return (false);
	if (!isnormal(center.getRotation().z()))
		return (false);
	if (!isnormal(center.getRotation().w()))
		return (false);
	if (!isnormal(direction.x()))
		return (false);
	if (!isnormal(direction.y()))
		return (false);
	if (!isnormal(direction.z()))
		return (false);
	return (true);
}

RotationalModel::RotationalModel(btBoxTrack* track) :
	ManifoldModel(track) {
}

void RotationalModel::findParameters(ros::Publisher* pub) {
	cout << " begin RotationalModel::findParameters"<<endl;
	ManifoldModel::findParameters();

	if (track->obs_history.size()<3) {
		return;
	}

	double size_thresh = 0.03;

	sensor_msgs::PointCloud points;
	points.header = vis_utils_header;
	points.set_points_size(track->obs_history.size());
	points.set_channels_size(1);
	points.channels[0].name="rgb";
	geometry_msgs::Point32 p;
	vector<int> indices;
	points.channels[0].set_values_size(track->obs_history.size());
	for (size_t i = 0; i < track->obs_history.size(); i++) {
		btTransform tf = track->obs_history[i].tf;
		points.points[i].x = tf.getOrigin().x();
		points.points[i].y = tf.getOrigin().y();
		points.points[i].z = tf.getOrigin().z();
		int rgb=0xffffff;
		points.channels[0].values[i]=*(float*) &rgb;

		if( fabs(track->obs_history[i].w - w)<size_thresh &&
				fabs(track->obs_history[i].h - h)<size_thresh)
			indices.push_back(i);
	}
	cout << "RotationalModel::findParameters(), width and height, support "
			<< indices.size() << " / " <<track->obs_history.size() << endl;

	double dist_thresh = 0.03;
	if(vis_utils_cloud_pub!=NULL) {
		points.header = vis_utils_header;
		vis_utils_cloud_pub->publish(points);
	}
	// ***************** begin plane fitting
	// Create and initialize the SAC model
	sample_consensus::SACModelPlane model_plane;
	sample_consensus::RANSAC sacPlane(&model_plane, dist_thresh);
	sacPlane.setMaxIterations(100);

	if (indices.size()<3) {
		return;
	}

	model_plane.setDataSet(&points, indices);
	if (!sacPlane.computeModel()) {
		cout << "RotationalModel::findParameters() could not find plane!"
				<< endl;
		return;
	}
	vector<double> model_coeff;
	sacPlane.computeCoefficients(model_coeff); // Compute the model coefficients
	sacPlane.refineCoefficients(model_coeff); // Refine them using least-squares

	// Get the list of inliers
	vector<int> model_inliers;
	model_plane.selectWithinDistance(model_coeff, dist_thresh, model_inliers);
	cout << "RotationalModel::findParameters() model found, support "
			<< model_inliers.size() << " / " << indices.size() << endl;

	sensor_msgs::PointCloud projected_points;
	model_plane.projectPoints(model_inliers,model_coeff,projected_points);

	if(vis_utils_cloud_pub!=NULL) {
		projected_points.header = vis_utils_header;
		vis_utils_cloud_pub->publish(projected_points);
	}
	// ***************** end plane fitting


	// ***************** find transformation to xy plane
	btVector3 rx,ry,rz(model_coeff[0],model_coeff[1],model_coeff[2]);
	if(rz.closestAxis()==2) {
		rx = btVector3(1,0,0);
	} else {
		if(rz.closestAxis()==0) {
			rx = btVector3(0,1,0);
		} else {
			rx = btVector3(0,0,1);
		}
	}
	rx = (rx - rz.dot(rx)*rz).normalize();
	ry = rz.cross(rx);

	btMatrix3x3 R;
	R[0] = rx;
	R[1] = ry;
	R[2] = rz;
//	cout << PRINTVEC(rx)<<endl;
//	cout << PRINTVEC(ry)<<endl;
//	cout << PRINTVEC(rz)<<endl;
//	cout << rx.length()<<endl;
//	cout << ry.length()<<endl;
//	cout << rz.length()<<endl;
//	cout << rx.dot(ry)<<endl;
//	cout << ry.dot(rz)<<endl;
//	cout << rz.dot(rx)<<endl;
//	cout << PRINTVEC(rx.cross(ry))<<endl;
	btQuaternion q;
	R.getRotation(q);
	btVector3 t(projected_points.points[0].x,projected_points.points[0].y,projected_points.points[0].z);
	btTransform tf(q,-(R*t));

	sensor_msgs::PointCloud centered_points = projected_points;
	for(size_t i=0;i<centered_points.get_points_size();i++) {
		btVector3 v(projected_points.points[i].x,projected_points.points[i].y,projected_points.points[i].z);
		btVector3 w = tf * v;
//		cout << "proj: "<<PRINTVEC(v) << "\t centered:"<<PRINTVEC(w)<< endl;
//		cout << v.dot( rz ) << endl;
		centered_points.points[i].x = w.x();
		centered_points.points[i].y = w.y();
		centered_points.points[i].z = w.z();
	}
	if(vis_utils_cloud_pub!=NULL) {
		centered_points.header = vis_utils_header;
//		vis_utils_cloud_pub->publish(centered_points);
}

	// ***************** find transformation to xy plane (end)

	// ***************** find circle in point cloud
	if (centered_points.points.size()<3) {
		return;
	}

	// Create and initialize the SAC model
	sample_consensus::SACModelCircle2D model_circle;
	sample_consensus::RANSAC sacCircle(&model_circle, dist_thresh);
	sacCircle.setMaxIterations(100);

	model_circle.setDataSet(&centered_points);
	if (!sacCircle.computeModel()) {
		cout << "RotationalModel::findParameters() could not find circle!"
				<< endl;
		return;
	}
	vector<double> model_coeff_circle;
	sacCircle.computeCoefficients(model_coeff_circle); // Compute the model coefficients
	sacCircle.refineCoefficients(model_coeff_circle); // Refine them using least-squares

	// Get the list of inliers
	vector<int> model_inliers_circle;
	model_circle.selectWithinDistance(model_coeff_circle, dist_thresh, model_inliers_circle);
	cout << "RotationalModel::findParameters() circle model found, support "
			<< model_inliers_circle.size() << " / " << centered_points.points.size() << endl;

	sensor_msgs::PointCloud circle_points = centered_points;
	model_circle.projectPoints(model_inliers_circle,model_coeff_circle,circle_points);

	if(vis_utils_cloud_pub!=NULL) {
		circle_points.header = vis_utils_header;
//		vis_utils_cloud_pub->publish(circle_points);
	}
	// ***************** find circle in point cloud (end)

	sensor_msgs::PointCloud circle_points2 = circle_points;
	for(size_t i=0;i<circle_points.get_points_size();i++) {
		btVector3 v(circle_points.points[i].x,circle_points.points[i].y,circle_points.points[i].z);
		btVector3 w = tf.inverse() * v;
		circle_points2.points[i].x = w.x();
		circle_points2.points[i].y = w.y();
		circle_points2.points[i].z = w.z();
	}
	if(vis_utils_cloud_pub!=NULL) {
		circle_points2.header = vis_utils_header;
//		vis_utils_cloud_pub->publish(circle_points2);
	}

	// compute center
	center = tf.inverse() * btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(model_coeff_circle[0],model_coeff_circle[1],0));

	// make obs_history[0].configuration==0
	cout << "conf[0]="<<getConfiguration(track->obs_history[0].tf)[0]<< endl;
	center = center * btTransform(btQuaternion(btVector3(0,0,1),-getConfiguration(track->obs_history[0].tf)[0] ),btVector3(0,0,0));
	cout << "conf[0]="<<getConfiguration(track->obs_history[0].tf)[0]<< endl;

	// radius
	radius =  btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(model_coeff_circle[2],0,0));

	// offset (t=0; r=avg)
	btQuaternion o =
		btTransform(btQuaternion(btVector3(0,0,1),-getConfiguration(track->obs_history[0].tf)[0] ),btVector3(0,0,0)) *
		track->obs_history[0].tf.getRotation();
	for (size_t j = 1; j < track->obs_history.size(); j++) {
		btQuaternion o2 =
			btTransform(btQuaternion(btVector3(0,0,1),-getConfiguration(track->obs_history[j].tf)[0] ),btVector3(0,0,0)) *
			track->obs_history[j].tf.getRotation();
//		cout << o2.x() << " "<<o2.y()<<" "<<o2.z()<<" "<<o2.w() << endl;
		o = o.slerp( o2	, 1 / (j + 1.0));
	}
	offset = btTransform(o, btVector3(0,0,0));
	offset =
		btTransform(
				btTransform(btQuaternion(btVector3(0,0,1),-getConfiguration(track->obs_history[0].tf)[0] ),btVector3(0,0,0)) *
				(center.inverse() *track->obs_history[0].tf).getRotation(),
				btVector3(0,0,0));



	std::vector<std::pair<btVector3,
	    btVector3> > lines;
	lines.push_back( std::pair<btVector3,btVector3>(center*btVector3(0,0,0),center*btVector3(0,0,0.1)));
	lines.push_back( std::pair<btVector3,btVector3>(center*btVector3(0,0,0),center*btVector3(0,0.1,0)));
	lines.push_back( std::pair<btVector3,btVector3>(center*btVector3(0,0,0),center*btVector3(0.1,0,0)));
	lines.push_back( std::pair<btVector3,btVector3>(center*btVector3(0,0,0),center*radius*btVector3(0,0,0)));
	lines.push_back( std::pair<btVector3,btVector3>(center*radius*btVector3(0,0,0),center*radius*btVector3(0,0,0.1)));
	lines.push_back( std::pair<btVector3,btVector3>(center*radius*btVector3(0,0,0),center*radius*btVector3(0,0.1,0)));
	lines.push_back( std::pair<btVector3,btVector3>(center*radius*btVector3(0,0,0),center*radius*btVector3(0.1,0,0)));
	visualizeLines(lines,800,1.0,0,0);
	cout << PRINTVEC(lines[0].first) << endl;
	cout << PRINTVEC(lines[0].second) << endl;
}

size_t RotationalModel::getDOFs() {
	return (1);
}

std::vector<double> RotationalModel::getConfiguration(btTransform transform) {
	std::vector<double> q;
	btTransform rel = center.inverseTimes(transform);
	q.push_back( -atan2(rel.getOrigin().y(),rel.getOrigin().x()) );
	return (q);
}

btTransform RotationalModel::getPrediction(std::vector<double> configuration) {
	return center * btTransform(btQuaternion(btVector3(0, 0, 1),
			-configuration[0]), btVector3(0, 0, 0)) * radius * offset;
}

bool RotationalModel::isValid() {
	return (true);
}

}
