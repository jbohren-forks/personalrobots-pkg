/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Marius Muja

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <point_cloud_mapping/cloud_io.h>

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>


#include "robot_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <string>

using namespace std;
using namespace robot_msgs;

USING_PART_OF_NAMESPACE_EIGEN


#include <tf/tfMessage.h>



class RectifyImage {

	robot_msgs::PointCloud point_cloud;
	IplImage* image;

	tf::TransformListener *tf_;
	ros::Node& node_;

	Matrix3f K;
	Matrix3f K_;


public:

	RectifyImage(ros::Node &node) : node_(node)
	{
		tf_ = new tf::TransformListener(node);

		K << 4763.3037100000001, 0.0, 1223.5, 0.0, 4783.6466700000001, 1024.5, 0.0, 0.0, 1.0;
//		K_ << 1.0/4870.0752, 0, -1223.5/4870.0752, 0, 1.0/4871.80719, -1024.5/4871.80719, 0, 0, 1.0;

        node_.advertise<visualization_msgs::Marker>("visualization_marker", 1);


	}

	bool fitSACLine(PointCloud& points, vector<int> indices, vector<double> &coeff, double dist_thresh, int min_pts, vector<Point32> &line_segment)
	{
		Point32 minP, maxP;

		// Create and initialize the SAC model
		sample_consensus::SACModelLine *model = new sample_consensus::SACModelLine ();
		sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
		sac->setMaxIterations (100);
		model->setDataSet (&points, indices);

		if(sac->computeModel())
		{
			if((int) sac->getInliers().size() < min_pts) {
				coeff.resize (0);
				return (false);
			}
			sac->computeCoefficients (coeff);

			Point32 minP, maxP;
			cloud_geometry::statistics::getLargestDiagonalPoints(points, sac->getInliers(), minP, maxP);
			line_segment.push_back(minP);
			line_segment.push_back(maxP);


			fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", (int)sac->getInliers ().size (), coeff[0], coeff[1], coeff[2], coeff[3]);
		}

		delete sac;
		delete model;
		return true;
	}

	double squaredPointDistance(Point32 p1, Point32 p2)
	{
		return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
	}

	/**
	* Assuming that the base laser is near the wall, get closest point in order to compute
	* region near the point where to fit a line.
	* @param cloud
	* @return
	*/
	Point32 findPointAhead(const PointCloud& cloud)
	{
		Point32 closest;
		float dist = 1e10;

		for (size_t i=0; i<cloud.get_pts_size(); ++i) {
			Point32 crt = cloud.pts[i];
			if (fabs(crt.x)<1e-10 && fabs(crt.y)<1e-10 && fabs(crt.z)<1e-10) continue;
			float crt_dist = fabs(crt.y);

			if (dist>crt_dist) {
				closest = crt;
				dist = crt_dist;
			}
		}

		return closest;
	}


	PointCloud getWallCloud(PointCloud laser_cloud, Point32 point, double distance)
	{
		PointCloud result;
		result.header.frame_id = laser_cloud.header.frame_id;
		result.header.stamp = laser_cloud.header.stamp;

		double d = distance*distance;
		for (size_t i=0; i<laser_cloud.get_pts_size(); ++i) {
			if (squaredPointDistance(laser_cloud.pts[i],point)<d) {
				result.pts.push_back(laser_cloud.pts[i]);
			}
		}

		return result;
	}


	void read_data(char* prefix)
	{

		char filename[100];
		sprintf(filename, "base_laser%s.pcd", prefix);

		printf("Trying to load file: %s\n", filename);
		cloud_io::loadPCDFile(filename, point_cloud);
		printf("Loaded %d points\n",point_cloud.get_pts_size());
		sprintf(filename, "frame%s.jpg", prefix);
		printf("Trying to load file: %s\n", filename);
		image = cvLoadImage(filename);
		printf("Image loaded\n");


		cvNamedWindow("test",0);
		cvShowImage("test",image);
		cvWaitKey(50);

	}

    void showPoint(robot_msgs::PointStamped point, int id)
    {
    	visualization_msgs::Marker marker;
        marker.header.frame_id = point.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.id = id ;
        marker.ns = "rectify_image";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        node_.publish("visualization_marker", marker);
    }

    void showLineMarker(const vector<Point32>& line_segment)
    {
    	visualization_msgs::Marker marker;
    	marker.header.frame_id = "base_laser";
    	marker.header.stamp = ros::Time((uint64_t)0ULL);
    	marker.ns = "rectify_image";
    	marker.id = 102;
    	marker.type = visualization_msgs::Marker::LINE_STRIP;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.orientation.w = 1.0;
    	marker.scale.x = 0.01;
    	marker.color.a = 1.0;
    	marker.color.g = 1.0;
    	marker.set_points_size(2);

    	marker.points[0].x = line_segment[0].x;
    	marker.points[0].y = line_segment[0].y;
    	marker.points[0].z = line_segment[0].z;

    	marker.points[1].x = line_segment[1].x;
    	marker.points[1].y = line_segment[1].y;
    	marker.points[1].z = line_segment[1].z;

    	node_.publish( "visualization_marker", marker );
    }



    void print_point(PointStamped p)
    {
    	printf("Point is: (%f,%f,%f)\n",p.point.x,p.point.y,p.point.z);
    }

	void compute_wall_frame()
	{
		printf("Finding point ahead");
		Point32 closest = findPointAhead(point_cloud);
		printf("(%f,%f,%f)\n", closest.x,closest.y,closest.z);

		printf("Filtering point cloud\n");
		PointCloud wall_cloud = getWallCloud(point_cloud,closest,0.2);


		// fit a line in the outlet cloud
		vector<int> indices(wall_cloud.pts.size());
		for (size_t i=0;i<wall_cloud.get_pts_size();++i) {
			indices[i] = i;
		}
		vector<double> coeff(4);	// line coefficients

		double dist_thresh = 0.01;
		int min_pts = 20;

		vector<Point32> line_segment;
		printf("Finding wall orientation\n");
		if ( !fitSACLine(wall_cloud, indices, coeff, dist_thresh, min_pts, line_segment) ) {
			printf("Cannot find line in laser scan, aborting...\n");
			return;
		}


//		showLineMarker(line_segment);

		PointStamped base_p1;
		base_p1.header.stamp = ros::Time();
		base_p1.header.frame_id = "base_laser";
		base_p1.point.x = line_segment[0].x;
		base_p1.point.y = line_segment[0].y;
		base_p1.point.z = line_segment[0].z;
		PointStamped prosilica_p1;

		PointStamped base_p2;
		base_p2.header.stamp = ros::Time();
		base_p2.header.frame_id = "base_laser";
		base_p2.point.x = line_segment[1].x;
		base_p2.point.y = line_segment[1].y;
		base_p2.point.z = line_segment[1].z;

//		showPoint(base_p1,1);
//		showPoint(base_p2,2);

		PointStamped prosilica_p2;

		tf_->transformPoint("high_def_optical_frame", base_p1, prosilica_p1);
		tf_->transformPoint("high_def_optical_frame", base_p2, prosilica_p2);


		print_point(prosilica_p1);
		print_point(prosilica_p2);

		Vector3f p1 = Vector3f(prosilica_p1.point.x,prosilica_p1.point.y,prosilica_p1.point.z);
		Vector3f p2 = Vector3f(prosilica_p2.point.x,prosilica_p2.point.y,prosilica_p2.point.z);

		cout << "Point 1 is: (" << p1 << ")" << endl;
		cout << "Point 2 is: (" << p2 << ")" << endl;

		p1 = K*p1;
		p2 = K*p2;

		p1 /= p1.z();
		p2 /= p2.z();


		CvPoint im_p1 = cvPoint(p1.x(),p1.y());
		CvPoint im_p2 = cvPoint(p2.x(),p2.y());



		cout << "Point 1 is: (" << p1 << ")" << endl;
		cout << "Point 2 is: (" << p2 << ")" << endl;


		double distance = sqrt(squaredPointDistance(line_segment[0],line_segment[1]));

		printf("Distance: %f\n", distance);

		base_p1.point.z += distance;
		base_p2.point.z += distance;

		tf_->transformPoint("high_def_optical_frame", base_p1, prosilica_p1);
		tf_->transformPoint("high_def_optical_frame", base_p2, prosilica_p2);

		print_point(prosilica_p1);
		print_point(prosilica_p2);

		Vector3f p3 = Vector3f(prosilica_p1.point.x,prosilica_p1.point.y,prosilica_p1.point.z);
		Vector3f p4 = Vector3f(prosilica_p2.point.x,prosilica_p2.point.y,prosilica_p2.point.z);

		p3 = K*p3;
		p4 = K*p4;

		p3 /= p3.z();
		p4 /= p4.z();

		CvPoint im_p3 = cvPoint(p3.x(),p3.y());
		CvPoint im_p4 = cvPoint(p4.x(),p4.y());

		cvLine(image, im_p1, im_p2, CV_RGB(255,0,0), 2);
//		cvLine(image, im_p3, im_p4, CV_RGB(0,255,0), 2);
//		cvLine(image, im_p1, im_p3, CV_RGB(0,0,255), 2);
//		cvLine(image, im_p2, im_p4, CV_RGB(0,255,255), 2);

		cvShowImage("test",image);

		cout << "Point 3 is: (" << p3 << ")" << endl;
		cout << "Point 4 is: (" << p4 << ")" << endl;

		IplImage* rectified = cvCloneImage(image);

		CvPoint2D32f objPts[4], imgPts[4];

		objPts[0].x = 1000; objPts[0].y = 500;
		objPts[1].x = 1500; objPts[1].y = 500;
		objPts[2].x = 1000; objPts[2].y = 1000;
		objPts[3].x = 1500; objPts[3].y = 1000;

		imgPts[0].x = p4.x(); imgPts[0].y = p4.y();
		imgPts[1].x = p3.x(); imgPts[1].y = p3.y();
		imgPts[2].x = p2.x(); imgPts[2].y = p2.y();
		imgPts[3].x = p1.x(); imgPts[3].y = p1.y();



		CvMat *H3 = cvCreateMat(3,3,CV_32F);
		float* f = H3->data.fl;

		cvGetPerspectiveTransform(objPts,imgPts,H3);


		printf("%f %f %f\n", f[0],f[1],f[2]);
		printf("%f %f %f\n", f[3],f[4],f[5]);
		printf("%f %f %f\n", f[6],f[7],f[8]);

		cvWarpPerspective(image,rectified, H3,
				CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS );

		cvNamedWindow("Rectified Image",0);

		cvShowImage("Rectified Image", rectified);


		cvWaitKey(0);

//
//		PoseStamped wall_pose;
//
//		// fill the outlet pose
//		wall_pose.header.frame_id = point_cloud.header.frame_id;
//		wall_pose.header.stamp = point_cloud.header.stamp;
//
//		btVector3 position(closest.x,closest.y,closest.z);
//		btVector3 up(0,0,1);
//		btVector3 left(line_segment[1].x-line_segment[0].x,line_segment[1].y-line_segment[0].y,line_segment[1].z-line_segment[0].z);
//		btVector3 normal = left.cross(up).normalized();
//
//
//		btMatrix3x3 rotation;
//		rotation[0] = normal; // x
//		rotation[1] = left; // y
//		rotation[2] = up;     // z
//		rotation = rotation.transpose();
//		btQuaternion orientation;
//		rotation.getRotation(orientation);
//		tf::Transform pose(orientation, position);
//
//
//
//
//
//		tf::Point point(-1,0,0);
//		tf::Point new_origin = pose*point;
//
//		pose.setOrigin(new_origin);
//
////		node_.advertise<tf::tfMessage>("tf_message",1);
////
////		tf::tfMessage message;
////
////		robot_msgs::TransformStamped transform;
////		transform.header.stamp = ros::Time();
////		transform.header.frame_id = "wall";
////		transform.parent_id = "base_laser";
////		transform.transform.translation.x = pose.getOrigin().x();
////		transform.transform.translation.y = pose.getOrigin().y();
////		transform.transform.translation.z = pose.getOrigin().z();
////
////		transform.transform.rotation.x = pose.getRotation().x();
////		transform.transform.rotation.y = pose.getRotation().y();
////		transform.transform.rotation.z = pose.getRotation().z();
////		transform.transform.rotation.w = pose.getRotation().w();
////
////		message.transforms.push_back(transform);
////
////		node_.publish("tf_message",message);
//
////		tf::Stamped<tf::Transform> new_frame(pose, ros::Time(), "wall", "base_laser");
////		tf_->setTransform(new_frame);
//
//		printf("Pose computed is: origin (%f,%f,%f), orientation (%f,%f,%f,%f)\n",
//				pose.getOrigin()[0],pose.getOrigin()[1],pose.getOrigin()[2],
//				pose.getRotation()[0],pose.getRotation()[1],pose.getRotation()[2],pose.getRotation()[3]);


	}



	void rectify_image()
	{

		tf_->setExtrapolationLimit(ros::DURATION_MAX);

		tf::Stamped<tf::Transform> camera;
		tf_->lookupTransform("base_laser", "high_def_optical_frame", ros::Time(), camera);

		btMatrix3x3 rotation = camera.getBasis();

		Matrix3f R;


		R << rotation[0][0], rotation[0][1], rotation[0][2],
			rotation[1][0], rotation[1][1],rotation[1][2],
			rotation[2][0], rotation[2][1],rotation[2][2];

		cout << "K:" << endl << K << endl;
		cout << "R:" << endl << R << endl;
		cout << "K_:" << endl << K_ << endl;


		cout << "K*R*K_:" << endl << K*R*K_ << endl;

		Matrix3f H2;
		H2 = K*R*K_;

		float H_[9];
		H_[0] = H2(0,0);
		H_[1] = H2(0,1);
		H_[2] = H2(0,2);

		H_[3] = H2(1,0);
		H_[4] = H2(1,1);
		H_[5] = H2(1,2);

		H_[6] = H2(2,0);
		H_[7] = H2(2,1);
		H_[8] = H2(2,2);

		CvMat H;
		cvInitMatHeader(&H, 3, 3, CV_32FC1, H_);


		IplImage* rectified = cvCloneImage(image);


		CvPoint2D32f objPts[4], imgPts[4];

		objPts[0].x = 1500; objPts[0].y = 1500;
		objPts[1].x = 2000; objPts[1].y = 1500;
		objPts[2].x = 1500; objPts[2].y = 2000;
		objPts[3].x = 2000; objPts[3].y = 2000;

		imgPts[0].x = 1536; imgPts[0].y = 1434;
		imgPts[1].x = 2048; imgPts[1].y = 1476;
		imgPts[2].x = 1530; imgPts[2].y = 1732;
		imgPts[3].x = 2010; imgPts[3].y = 1766;



		CvMat *H3 = cvCreateMat(3,3,CV_32F);
		float* f = H3->data.fl;

		cvGetPerspectiveTransform(objPts,imgPts,H3);

		f[8] = 0.5;

		printf("%f %f %f\n", f[0],f[1],f[2]);
		printf("%f %f %f\n", f[3],f[4],f[5]);
		printf("%f %f %f\n", f[6],f[7],f[8]);

		cvWarpPerspective(image,rectified, &H,
				CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS );

		cvNamedWindow("Rectified Image",0);

		cvShowImage("Rectified Image", rectified);

		cvWaitKey(0);

	}


	void run(char* prefix)
	{
		read_data(prefix);
		compute_wall_frame();

//		rectify_image();
	}
};

int main(int argc, char** argv)
{
	if (argc<2) {
		printf("Usage: %s prefix", argv[0] );
	}

	ros::init(argc, argv);

	ros::Node ros_node ("rectify_hires_image");
	RectifyImage rectify_image(ros_node);

	rectify_image.run(argv[1]);

	return 0;
}
