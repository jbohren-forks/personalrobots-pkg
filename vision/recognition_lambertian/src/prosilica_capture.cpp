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


#include <string>

#include "ros/node.h"
#include "opencv_latest/CvBridge.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <tf/transform_listener.h>

#include "joy/Joy.h"
#include "prosilica_cam/PolledImage.h"

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

static char windowName[] = "Captured image";
static char rectifiedWindowName[] = "Rectified image";


using namespace std;
using namespace robot_msgs;

class ProsilicaCapture {

	image_msgs::CvBridge bridge;
	IplImage* image;
	IplImage* rectified;

	prosilica_cam::PolledImage::Request req;
	prosilica_cam::PolledImage::Response res;

	unsigned int index;

	robot_msgs::PointCloud base_cloud_;
	robot_msgs::PointCloud base_cloud_fetch_;
	string base_laser_topic_;

	ros::Node& node_;

	boost::mutex clound_point_mutex;
	boost::condition_variable cloud_point_cv;
	bool capture_cloud_point_;

	tf::TransformListener *tf_;

	Matrix3f K;

	joy::Joy joy;
	int capture_button_;
	bool display_;

public:
	ProsilicaCapture(ros::Node& node) : index(0), node_(node)
	{
		tf_ = new tf::TransformListener(node);

		image = NULL;
		rectified = NULL;

		node.param<string>("~base_laser", base_laser_topic_, "base_scan_marking");
		node.param("~capture_button", capture_button_, 0);
		node.param("~display", display_, false);
		node.subscribe<robot_msgs::PointCloud,ProsilicaCapture>(base_laser_topic_, base_cloud_, &ProsilicaCapture::laser_callback, this, 1);
		node.subscribe<joy::Joy,ProsilicaCapture>("joy", joy, &ProsilicaCapture::joy_callback, this, 1);

		if (display_) {
			cvNamedWindow(windowName, 0);
			cvNamedWindow(rectifiedWindowName,0);
		}
	}

	void captureFromProsilica(int timeout)
	{
		req.timeout_ms = timeout;
		ROS_INFO("Capturing prosilica image");
		if (!ros::service::call("prosilica/poll", req, res)) {
			ROS_ERROR("Service call failed");
			image = NULL;
		}
		else {
			if (!bridge.fromImage(res.image, "bgr")) {
				ROS_ERROR("CvBridge::fromImage failed");
				image = NULL;
			}
			else {
				image = bridge.toIpl();
				K << res.cam_info.K[0], res.cam_info.K[1], res.cam_info.K[2],
					res.cam_info.K[3],res.cam_info.K[4],res.cam_info.K[5],
					res.cam_info.K[6],res.cam_info.K[7],res.cam_info.K[8];
			}
		}
	}

	void laser_callback()
	{
		if (!capture_cloud_point_) {
			return;
		}
		boost::lock_guard<boost::mutex> lock(clound_point_mutex);
		capture_cloud_point_ = false;
		base_cloud_ = base_cloud_fetch_;

		cloud_point_cv.notify_all();
	}



	void captureFromBaseLaser()
	{
		boost::unique_lock<boost::mutex> lock(clound_point_mutex);
		capture_cloud_point_ = true;
		// waiting for the cloud point
		ROS_INFO("Waiting for base laser");
		while (capture_cloud_point_) {
			cloud_point_cv.wait(lock);
		}
	}

	void capture()
	{
		ROS_INFO("Capturing data");
		captureFromProsilica(100);
		captureFromBaseLaser();

		rectifyProsilicaImage();
	}


	void joy_callback()
	{
		joy.lock();
		if (capture_button_<(int)joy.buttons.size() && joy.buttons[capture_button_]) {
			capture();
			saveData();
		}
		joy.unlock();
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


			ROS_INFO ("> Found a model supported by %d inliers: [%g, %g, %g, %g]", (int)sac->getInliers ().size (), coeff[0], coeff[1], coeff[2], coeff[3]);
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

	/**
	 * This method rectifies an image captured from the prosilica camera by using the
	 * base laser to determine the orientation of the wall.
	 */
	void rectifyProsilicaImage()
	{
		ROS_INFO("Finding wall point in front of the robot, the robot should be facing to the wall at this point.");
		Point32 start_point = findPointAhead(base_cloud_);
//		printf("(%f,%f,%f)\n", start_point.x,start_point.y,start_point.z);

		ROS_INFO("Filtering point cloud");
		PointCloud wall_cloud = getWallCloud(base_cloud_,start_point,0.2);

		// fit a line in the outlet cloud
		vector<int> indices(wall_cloud.pts.size());
		for (size_t i=0;i<wall_cloud.get_pts_size();++i) {
			indices[i] = i;
		}
		vector<double> coeff(4);	// line coefficients

		double dist_thresh = 0.01;
		int min_pts = 20;

		vector<Point32> line_segment;
		ROS_INFO("Computing wall orientation");
		if ( !fitSACLine(wall_cloud, indices, coeff, dist_thresh, min_pts, line_segment) ) {
			ROS_ERROR("Cannot fit line in laser scan, aborting...");
			return;
		}


		PointStamped base_p1, base_p2;
		PointStamped prosilica_p1,prosilica_p2;

		base_p1.header.stamp = ros::Time();
		base_p1.header.frame_id = base_cloud_.header.frame_id;
		base_p1.point.x = line_segment[0].x;
		base_p1.point.y = line_segment[0].y;
		base_p1.point.z = line_segment[0].z;

		base_p2.header.stamp = ros::Time();
		base_p2.header.frame_id = base_cloud_.header.frame_id;;
		base_p2.point.x = line_segment[1].x;
		base_p2.point.y = line_segment[1].y;
		base_p2.point.z = line_segment[1].z;

		// need the points in the prosilica frame
		tf_->transformPoint("high_def_optical_frame", base_p1, prosilica_p1);
		tf_->transformPoint("high_def_optical_frame", base_p2, prosilica_p2);

		// project the points into the image plane
		Vector3f p1 = Vector3f(prosilica_p1.point.x,prosilica_p1.point.y,prosilica_p1.point.z);
		p1 = K*p1;
		p1 /= p1.z();


		Vector3f p2 = Vector3f(prosilica_p2.point.x,prosilica_p2.point.y,prosilica_p2.point.z);
		p2 = K*p2;
		p2 /= p2.z();

		// find another 2 points higher up on the wall
		double distance = sqrt(squaredPointDistance(line_segment[0],line_segment[1]));
		base_p1.point.z += distance;
		base_p2.point.z += distance;

		// need the points in the prosilica frame
		tf_->transformPoint("high_def_optical_frame", base_p1, prosilica_p1);
		tf_->transformPoint("high_def_optical_frame", base_p2, prosilica_p2);


		Vector3f p3 = Vector3f(prosilica_p1.point.x,prosilica_p1.point.y,prosilica_p1.point.z);
		p3 = K*p3;
		p3 /= p3.z();

		Vector3f p4 = Vector3f(prosilica_p2.point.x,prosilica_p2.point.y,prosilica_p2.point.z);
		p4 = K*p4;
		p4 /= p4.z();

		if (rectified) {
			cvReleaseImage(&rectified);
		}
		rectified = cvCloneImage(image);
		CvPoint2D32f objPts[4], imgPts[4];

		objPts[0].x = 1000; objPts[0].y = 500;
		objPts[1].x = 1500; objPts[1].y = 500;
		objPts[2].x = 1000; objPts[2].y = 1000;
		objPts[3].x = 1500; objPts[3].y = 1000;

		imgPts[0].x = p4.x(); imgPts[0].y = p4.y();
		imgPts[1].x = p3.x(); imgPts[1].y = p3.y();
		imgPts[2].x = p2.x(); imgPts[2].y = p2.y();
		imgPts[3].x = p1.x(); imgPts[3].y = p1.y();

		// compute perspective transformation
		CvMat *H = cvCreateMat(3,3,CV_32F);
		cvGetPerspectiveTransform(objPts,imgPts,H);
		cvWarpPerspective(image,rectified, H,
				CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS );

		cvReleaseMat(&H);

		if (display_) {
			cvShowImage(rectifiedWindowName, rectified);
		}
	}

	void saveData()
	{
		ROS_INFO("Saving data");
		if (image!=NULL) {
			char filename[160];
			// saving image
			sprintf(filename, "frame%04u.jpg", index);
			cvSaveImage( filename, rectified );
			ROS_INFO("Saved image %s", filename);

			sprintf(filename, "rectified_frame%04u.jpg", index);
			cvSaveImage( filename, rectified );
			ROS_INFO("Saved image %s", filename);

			sprintf(filename, "base_laser%04u.jpg", index);
			cloud_io::savePCDFile (filename, base_cloud_, true);
			ROS_INFO("Saved base point cloud %s", filename);
			index++;
		}

	}

	void spin()
	{

		while (true)
		{
			int k = cvWaitKey(0);
			switch( (char) k)
			{
			case 'c':
				capture();
				if (image)
					cvShowImage(windowName, image);
				break;
			case 's':
				saveData();
				break;
			}
		}

	}



};




int main(int argc, char** argv)
{
	ros::init(argc, argv);
	ros::Node n("prosilica_capture");

	ProsilicaCapture capture(n);
	capture.spin();


}
