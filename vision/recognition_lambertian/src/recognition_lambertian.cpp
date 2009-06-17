/*********************************************************************
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

#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <algorithm>
#include <libgen.h> // for dirname()


#include "opencv_latest/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/projections.h>



#include "ros/ros.h"
//#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"

#include <string>

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include "topic_synchronizer/topic_synchronizer.h"

#include "CvStereoCamModel.h"

#include <boost/thread.hpp>

#include "chamfer_matching/chamfer_matching.h"
//#include "flann.h"

using namespace robot_msgs;
using namespace std;


void on_templates_no(int);
void on_edges_high(int);


#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)




class TopicSynchronizer
{
	int expected_count_;
	int count_;
	ros::Time time_;
	boost::function<void ()> callback_;

public:

	template <typename T>
	TopicSynchronizer(void(T::*fp)(), T* obj) : callback_(boost::bind(fp,obj))
	{
		init();
	}

	TopicSynchronizer(void (*fp)()) : callback_(fp)
	{
		init();
	}

	void setCount(int count)
	{
		expected_count_ = count;
	}

	void init()
	{
		count_ = 0;
	}

	void update(const ros::Time& time)
	{
		if (count_==0 || time>time_) {
			time_ = time;
			count_ = 0;
		}

		if (time==time_) {
			count_++;
		}

		if (count_==expected_count_) {
			callback_();
		}
	}
};



class RecognitionLambertian
{
public:

	ros::NodeHandle nh_;

	image_msgs::ImageConstPtr limage;
	image_msgs::ImageConstPtr rimage;
	image_msgs::ImageConstPtr dimage;
	image_msgs::StereoInfoConstPtr stinfo;
	image_msgs::DisparityInfoConstPtr dispinfo;
	image_msgs::CamInfo rcinfo;
	image_msgs::CamInfo lcinfo;
	image_msgs::CvBridge lbridge;
	image_msgs::CvBridge rbridge;
	image_msgs::CvBridge dbridge;


	ros::Subscriber left_image_sub_;
	ros::Subscriber right_image_sub_;
	ros::Subscriber disparity_sub_;
	ros::Subscriber cloud_sub_;


	ros::Publisher object_pub_;
	ros::Publisher objects_pub_;
	ros::Publisher marker_pub_;

	robot_msgs::PointCloudConstPtr cloud;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

//	TopicSynchronizer<RecognitionLambertian> sync;

	boost::mutex cv_mutex;
	boost::condition_variable images_ready;

	tf::TransformListener tf_;

	TopicSynchronizer sync_;


	// minimum height to look at (in base_link frame)
	double min_height;
	// maximum height to look at (in base_link frame)
	double max_height;
	// no. of frames to detect handle in
	int frames_no;
	// display stereo images ?
	bool display;

	int templates_no;
	int edges_high;


	ChamferMatching* cm;

	RecognitionLambertian()
	: left(NULL), right(NULL), disp(NULL), disp_clone(NULL), sync_(&RecognitionLambertian::syncCallback, this)
	{
		// define node parameters
		nh_.param("~min_height", min_height, 0.7);
		nh_.param("~max_height", max_height, 1.0);
		nh_.param("~frames_no", frames_no, 7);
		nh_.param("~display", display, false);
		string template_path;
		nh_.param<string>("~template_path", template_path,"templates.txt");

		templates_no = 10;
		edges_high = 170;

		if(display){
			cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("disparity_clone", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("edges",1);
			//        	cvCreateTrackbar("templates","edges",&templates_no, 7, &on_templates_no);
			//        	cvCreateTrackbar("edges_high","edges",&edges_high, 500, &on_edges_high);
		}

		// subscribe to topics
		left_image_sub_ = nh_.subscribe("stereo/left/image_rect", 1, &RecognitionLambertian::leftImageCallback, this);
		right_image_sub_ = nh_.subscribe("stereo/right/image_rect", 1, &RecognitionLambertian::rightImageCallback, this);
		disparity_sub_ = nh_.subscribe("stereo/disparity", 1, &RecognitionLambertian::disparityImageCallback, this);
		cloud_sub_ = nh_.subscribe("stereo/cloud", 1, &RecognitionLambertian::cloudCallback, this);

		sync_.setCount(4);

		// advertise topics
		objects_pub_ = nh_.advertise<PointCloud> ("~objects", 1);
		object_pub_ = nh_.advertise<PointCloud> ("~object", 1);
//		advertise<PointCloud> ("~outliers", 1);
//		advertise<PointCloud> ("~inliers", 1);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);


		cm = new ChamferMatching();
		loadTemplates(template_path);

	}

	~RecognitionLambertian()
	{
		delete cm;
	}

private:

	void syncCallback()
	{
		runRecognitionLambertian();
		cvWaitKey(100);
	}

	void leftImageCallback(const image_msgs::Image::ConstPtr& image)
	{
		limage = image;
		if(lbridge.fromImage(*limage, "bgr")) {
			left = lbridge.toIpl();
		}
		sync_.update(image->header.stamp);
	}

	void rightImageCallback(const image_msgs::Image::ConstPtr& image)
	{
		rimage = image;
		if(rbridge.fromImage(*rimage, "bgr")) {
			right = rbridge.toIpl();
		}
		sync_.update(image->header.stamp);
	}

	void disparityImageCallback(const image_msgs::Image::ConstPtr& image)
	{
		dimage = image;
		if(dbridge.fromImage(*dimage)) {
			disp = dbridge.toIpl();
		}
		sync_.update(image->header.stamp);
	}

	void cloudCallback(const robot_msgs::PointCloud::ConstPtr& point_cloud)
	{
		printf("Got point cloud\n");
		cloud = point_cloud;
		sync_.update(point_cloud->header.stamp);
	}


	void trimSpaces( string& str)
	{
		size_t startpos = str.find_first_not_of(" \t\n"); // Find the first character position after excluding leading blank spaces
		size_t endpos = str.find_last_not_of(" \t\n"); // Find the first character position from reverse af

		// if all spaces or empty return an empty string
		if(( string::npos == startpos ) || ( string::npos == endpos)) {
			str = "";
		}
		else {
			str = str.substr( startpos, endpos-startpos+1 );
		}
	}

	void loadTemplates(string path)
	{

		FILE* fin = fopen(path.c_str(),"r");

		if (fin==NULL) {
			ROS_ERROR("Cannot open template list: %s", path.c_str());
			exit(1);
		}

		char* path_ptr = strdup(path.c_str());
		string dir_path = dirname(path_ptr);

		char line[512];
		while (fgets(line,512, fin)) {

			string template_file = line;
			trimSpaces(template_file);
			printf("template file: %s\n",template_file.c_str());
			string template_path = dir_path + "/" + template_file;

			if (!template_file.empty()) {
				IplImage* templ = cvLoadImage(template_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
				if (!templ) {
					ROS_ERROR("Cannot load template image: %s", template_path.c_str());
					exit(1);
				}
				cm->addTemplateFromImage(templ, 0.2);
				cvReleaseImage(&templ);
			}
		}

		free(path_ptr);
		fclose(fin);

	}


	bool fitSACPlane (const PointCloud& points, const vector<int> &indices,  // input
			vector<int> &inliers, vector<double> &coeff,  // output
			double dist_thresh, int min_points_per_model)
	{
		// Create and initialize the SAC model
		sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
		sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
		sac->setMaxIterations (100);
		model->setDataSet ((PointCloud*)&points, indices);

		// Search for the best plane
		if (sac->computeModel ()) {
			sac->computeCoefficients (coeff);                              // Compute the model coefficients
			sac->refineCoefficients (coeff);                             // Refine them using least-squares

			// Get the list of inliers
			model->selectWithinDistance (coeff, dist_thresh, inliers);

			if ((int)inliers.size()<min_points_per_model) {
				return false;
			}

			Point32 viewpoint;
			viewpoint.x = 0;
			viewpoint.y = 0;
			viewpoint.z = 0;
			// Flip the plane normal towards the viewpoint
			cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at(inliers[0]), viewpoint);

			ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (), coeff[0], coeff[1], coeff[2], coeff[3]);
		}
		else {
			ROS_ERROR ("Could not compute a planar model for %d points.", indices.size());
			return false;
		}

		delete sac;
		delete model;
		return true;
	}


	void filterByZBounds(const PointCloud& pc, double zmin, double zmax, PointCloud& filtered_pc, PointCloud& filtered_outside)
	{
		vector<int> indices_remove;
		for (size_t i = 0;i<cloud->get_pts_size();++i) {
			if (cloud->pts[i].z>zmax || cloud->pts[i].z<zmin) {
				indices_remove.push_back(i);
			}
		}
		cloud_geometry::getPointCloudOutside (pc, indices_remove, filtered_pc);
		cloud_geometry::getPointCloud(pc, indices_remove, filtered_outside);
	}


	void clearFromImage(IplImage* disp_img, const PointCloud& pc)
	{
		int xchan = -1;
		int ychan = -1;
		for(size_t i = 0;i < pc.chan.size();++i){
			if(pc.chan[i].name == "x"){
				xchan = i;
			}
			if(pc.chan[i].name == "y"){
				ychan = i;
			}
		}

		if (xchan==-1 || ychan==-1) {
			ROS_ERROR("Cannot find image coordinates in the point cloud");
			return;
		}

		// remove plane points from disparity image
		for (size_t i=0;i<pc.get_pts_size();++i) {
			int x = pc.chan[xchan].vals[i];
			int y = pc.chan[ychan].vals[i];
			//			printf("(%d,%d)\n", x, y);
			CV_PIXEL(unsigned char, disp_img, x, y)[0] = 0;
		}
	}


	void filterTablePlane(vector<double>& coefficients, PointCloud& object_cloud, PointCloud& plane_cloud)
	{

		disp_clone = cvCloneImage(disp);

		PointCloud filtered_cloud;
		PointCloud filtered_outside;

		filterByZBounds(*cloud,0.1, 1.2 , filtered_cloud, filtered_outside );

		clearFromImage(disp, filtered_outside);

		vector<int> indices(filtered_cloud.get_pts_size());
		for (size_t i = 0;i<filtered_cloud.get_pts_size();++i) {
			indices[i] = i;
		}

		vector<int> inliers;
		double dist_thresh = 0.01; // in meters
		int min_points = 200;

		fitSACPlane(filtered_cloud, indices, inliers, coefficients, dist_thresh, min_points);

		cloud_geometry::getPointCloud(filtered_cloud, inliers, plane_cloud);
		cloud_geometry::getPointCloudOutside (filtered_cloud, inliers, object_cloud);

		clearFromImage(disp, plane_cloud);
	}


	PointCloud projectToPlane(const PointCloud& objects, const vector<double>& coefficients)
	{
		// clear "under the table" points
		vector<int> object_indices(objects.pts.size());
		for (size_t i=0;i<objects.get_pts_size();++i) {
			object_indices[i] = i;
		}

		PointCloud object_projections;
		object_projections.header.stamp = cloud->header.stamp;
		object_projections.header.frame_id = cloud->header.frame_id;

		cloud_geometry::projections::pointsToPlane(objects, object_indices, object_projections, coefficients);

		return object_projections;

	}


	/**
	* \brief Finds edges in an image
	* @param img
	*/
	void doChamferMatching(IplImage *img, const vector<CvPoint>& positions, const vector<float>& scales)
	{
		// edge detection
		IplImage *gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, gray, CV_RGB2GRAY);
		cvCanny(gray, gray, edges_high/2, edges_high);

		if (display) {
			cvShowImage("edges", gray);
		}


//		for (int i=0;i<scales.size();++i) {
//			printf("%f, ", scales[i]);
//		}
//		printf("\n");

		ChamferMatch match = cm->matchEdgeImage(gray, positions, scales);
		IplImage* left_clone = cvCloneImage(left);

		match.show(left, templates_no);



		if(display){
			// show filtered disparity
			cvShowImage("disparity", disp);
			cvShowImage("disparity_clone", disp_clone);
			// show left image
			cvShowImage("left", left);
			cvShowImage("right", right);
		}

		cvCopy(left_clone, left);
		cvReleaseImage(&left_clone);

		cvReleaseImage(&gray);
	}


	void findClusters(const PointCloud& cloud)
	{
		const int NUM_CLUSTERS = 10;

		int count = cloud.get_pts_size();
		CvMat* points = cvCreateMat( count, 3, CV_32FC1 );
		CvMat* clusters = cvCreateMat( count, 1, CV_32SC1 );

		for (int i=0;i<count;++i) {
			float* ptr = (float*)(points->data.ptr + i * points->step);
			ptr[0] = cloud.pts[i].x;
			ptr[1] = cloud.pts[i].y;
			ptr[2] = cloud.pts[i].z;
		}

		cvKMeans2(points, NUM_CLUSTERS, clusters, cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.005 ),10);


		// compute cluster sizes
		vector<int> cluster_size(count,0);
		for (int i=0;i<count;++i) {
			cluster_size[clusters->data.i[i]]++;
		}

		for (int i=0;i<NUM_CLUSTERS;++i) {
			printf("Cluster %d, size: %d\n", i, cluster_size[i]);
		}


		// visualize data
		visualization_msgs::Marker marker;
		marker.header.frame_id = cloud.header.frame_id;
		marker.header.stamp = cloud.header.stamp;
		marker.ns = "recognition_lambertian_clusters";
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.001;
		marker.scale.y = 0.001;
		marker.scale.z = 0.001;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;



		int* clusters_ptr = clusters->data.i;
		for (int i=0;i<NUM_CLUSTERS;++i) {
			printf("Publishing cluster: %d\n",i);
			marker.id = i;
			marker.points.clear();
			for (size_t j=0;j<cloud.get_pts_size();++j) {
				if (clusters_ptr[j]==i) {
					Point p;
					p.x = cloud.pts[j].x;
					p.y = cloud.pts[j].y;
					p.z = cloud.pts[j].z;
					marker.points.push_back(p);
					marker.color.r = (i+1)&1;
					marker.color.g = ((i+1)>>1)&1;
					marker.color.b = ((i+1)>>2)&1;
				}
			}
			marker_pub_.publish(marker);
		}


	}


	void addTableFrame(PointStamped origin, const vector<double>& plane)
	{

		btVector3 position(origin.point.x,origin.point.y,origin.point.z);

		btQuaternion orientation;
		btMatrix3x3 rotation;
		btVector3 z(plane[0],plane[1],plane[2]);
		btVector3 x(-plane[1],plane[0],0);
		x = x.normalized();
		btVector3 y = z.cross(x).normalized();
		rotation[0] = x; 	// x
		rotation[1] = y; 	// y
		rotation[2] = z; 	// z
		rotation = rotation.transpose();
		rotation.getRotation(orientation);

		tf::Transform tf_pose(orientation, position);

		// add wall_frame to tf
		tf::Stamped<tf::Pose> table_pose_frame(tf_pose, origin.header.stamp, "table_frame", origin.header.frame_id);

		tf_.setTransform(table_pose_frame);

//		tf::TransformBroadcaster broadcaster_;
//		broadcaster_.sendTransform(table_pose_frame);
	}



	void drawTableBBox(const PointCloud& cloud)
	{
		if (cloud.get_pts_size()==0) return;
		float xmin = cloud.pts[0].x;
		float xmax = cloud.pts[0].x;
		float ymin = cloud.pts[0].y;
		float ymax = cloud.pts[0].y;

		for (size_t i=1;i<cloud.get_pts_size();++i) {
			if (cloud.pts[i].x<xmin) xmin = cloud.pts[i].x;
			if (cloud.pts[i].x>xmax) xmax = cloud.pts[i].x;
			if (cloud.pts[i].y<ymin) ymin = cloud.pts[i].y;
			if (cloud.pts[i].y>ymax) ymax = cloud.pts[i].y;
		}

		// visualize data
		visualization_msgs::Marker marker;
		marker.header.frame_id = cloud.header.frame_id;
		marker.header.stamp = cloud.header.stamp;
		marker.ns = "recognition_lambertian_clusters";
		marker.id = 111;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.001;
		marker.scale.y = 0.001;
		marker.scale.z = 0.001;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker.points.resize(5);
		marker.points[0].x = xmin;
		marker.points[0].y = ymin;
		marker.points[0].z = 0;

		marker.points[1].x = xmin;
		marker.points[1].y = ymax;
		marker.points[1].z = 0;

		marker.points[2].x = xmax;
		marker.points[2].y = ymax;
		marker.points[2].z = 0;

		marker.points[3].x = xmax;
		marker.points[3].y = ymin;
		marker.points[3].z = 0;

		marker.points[4].x = xmin;
		marker.points[4].y = ymin;
		marker.points[4].z = 0;

		marker_pub_.publish(marker);

	}

	void showCluster(Point32 p, float radius, int idx, const ros::Time& stamp, float* color = NULL)
	{
		float default_color[] = { 1,1,0};

		if (color==NULL) {
			color = default_color;
		}

		visualization_msgs::Marker marker;
		marker.header.frame_id = "table_frame";
		marker.header.stamp = stamp;
		marker.ns = "recognition_lambertian_clusters";
		marker.id = 200+idx;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = p.x;
		marker.pose.position.y = p.y;
		marker.pose.position.z = p.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = radius*2;
		marker.scale.y = radius*2;
		marker.scale.z = 0;
		marker.color.a = 1.0;
		marker.color.r = color[0];
		marker.color.g = color[1];
		marker.color.b = color[2];

		marker_pub_.publish(marker);
	}

//	class NNIndexer
//	{
//		float* points;
//		FLANN_INDEX index;
//		const PointCloud& cloud_;
//	public:
//		NNIndexer(const PointCloud& cloud) : cloud_(cloud)
//		{
//			int size = cloud.get_pts_size();
//			points = new float[size*2];
//			for (int i=0;i<size;++i) {
//				points[2*i] = cloud.pts[i].x;
//				points[2*i+1] = cloud.pts[i].y;
//			}
//			FLANNParameters params;
//			params.algorithm = KDTREE;
//			params.trees = 1;
//			params.log_level = LOG_ERROR;
//			params.log_destination = NULL;
//			params.target_precision = -1;
//
//			float speedup = 0;
//			index = flann_build_index(points, size, 2, &speedup, &params);
//		}
//
//		~NNIndexer()
//		{
//			delete[] points;
//			flann_free_index(index, NULL);
//		}
//
//
//		int radiusSearch(const Point32& p, float radius, vector<int>& indices)
//		{
//			float q[2];
//			q[0] = p.x;
//			q[1] = p.y;
//			vector<float> dists(indices.size());
//			int nn_count = flann_radius_search(index, q, &indices[0], &dists[0], indices.size(), radius*radius, -1, NULL);
//
//			return nn_count;
//		}
//
//	};


	class NNGridIndexer
	{
		const PointCloud& cloud_;
		float xmin, xmax, ymin, ymax;
		float xd,yd;

		int resolution;

		float* grid;

	public:
		NNGridIndexer(const PointCloud& cloud) : cloud_(cloud)
		{
			xmin = cloud.pts[0].x;
			xmax = cloud.pts[0].x;
			ymin = cloud.pts[0].y;
			ymax = cloud.pts[0].y;
			for (size_t i=1;i<cloud.get_pts_size();++i) {
				if (cloud.pts[i].x<xmin) xmin = cloud.pts[i].x;
				if (cloud.pts[i].x>xmax) xmax = cloud.pts[i].x;
				if (cloud.pts[i].y<ymin) ymin = cloud.pts[i].y;
				if (cloud.pts[i].y>ymax) ymax = cloud.pts[i].y;
			}

			resolution = 600;
			xd = (xmax-xmin)/resolution;
			yd = (ymax-ymin)/resolution;

			grid = new float[resolution*resolution];
			memset(grid,0,resolution*resolution*sizeof(float));

			for (size_t i=0;i<cloud.get_pts_size();++i) {
				Point32 p = cloud.pts[i];

				int x = int((p.x-xmin)/xd+0.5);
				int y = int((p.y-ymin)/yd+0.5);

				float *ptr = grid+x*resolution+y;
				*ptr = max(p.z,*ptr);
			}
		}

		~NNGridIndexer()
		{
			delete[] grid;
		}


		int computeMean(const Point32& p, float radius, Point32& result)
		{
			int xc = int((p.x-xmin)/xd+0.5);
			int yc = int((p.y-ymin)/yd+0.5);

			int xoffs = int((radius/xd)+0.5);
			int yoffs = int((radius/yd)+0.5);

			float xmean = 0;
			float ymean = 0;
			int count = 0;

			for (int x=xc-xoffs;x<=xc+xoffs;++x) {
				for (int y=yc-yoffs;y<=yc+yoffs;++y) {
					if (x<0 || x>=resolution) continue;
					if (y<0 || y>=resolution) continue;
					if (double((x-xc)*(x-xc))/(xoffs*xoffs)+double((y-yc)*(y-yc))/(yoffs*yoffs)>=1) continue;
					if (grid[x*resolution+y]==0) continue;

					xmean += x;
					ymean += y;
					count ++;
				}
			}


			if (count==0) return 0;

			xmean /= count;
			ymean /= count;

			result.x = xmean*xd+xmin;
			result.y = ymean*yd+ymin;

			return count;
		}

	};


	Point32 computeMean(const PointCloud& pc, const vector<int>& indices, int count)
	{
		Point32 mean;
		mean.x = 0;
		mean.y = 0;

		for (int i=0;i<count;++i) {
			int j = indices[i];
			mean.x += pc.pts[j].x;
			mean.y += pc.pts[j].y;
		}

		mean.x /= count;
		mean.y /= count;

		return mean;
	}


	template<typename T>
	static double dist2D(const T& a, const T& b)
	{
		return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
	}


//	double meanShiftIteration(const PointCloud& pc, NNIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step, vector<int>& indices)
//	{
//		double total_dist = 0;
//		for (size_t i=0;i<centers.size();++i) {
//
//			int count = index.radiusSearch(centers[i], step, indices);
//			Point32 mean = computeMean(pc, indices, count);
//			double dist = dist2D(mean, centers[i]);
//			total_dist += dist;
//			means[i] = mean;
//		}
//		return total_dist;
//	}

	double meanShiftIteration2(const PointCloud& pc, NNGridIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step)
	{
		double total_dist = 0;
		for (size_t i=0;i<centers.size();++i) {

			Point32 mean;
			int count = index.computeMean(centers[i], step, mean);
			if (count==0) {
				printf("This should not happen\n");
			}
			double dist = dist2D(mean, centers[i]);
			total_dist += dist;
			means[i] = mean;
		}
		return total_dist;
	}

#define CLUSTER_RADIUS 0.15/2
#define MIN_OBJ_HEIGHT 0.06
#define MIN_OBJ_DIST 0.05


	void filterClusters(const PointCloud& cloud, vector<Point32>& centers, vector<Point32>& clusters)
	{
		vector<double> cluster_heights(centers.size(), 0);

		printf("Number of centers: %d\n", centers.size());

		// compute cluster heights
		for(size_t i=0;i<cloud.get_pts_size();++i) {
			for (size_t j=0;j<centers.size();++j) {
				if (dist2D(cloud.pts[i], centers[j])<CLUSTER_RADIUS*CLUSTER_RADIUS) {
					cluster_heights[j] = max(cluster_heights[j], (double)cloud.pts[i].z);
				}
			}
		}

		for (size_t i=0;i<centers.size();++i) {
			if (cluster_heights[i]>MIN_OBJ_HEIGHT) {

				bool duplicate = false;
				// check if duplicate cluster
				for (size_t j=0;j<clusters.size();++j) {
					if (dist2D(clusters[j],centers[i])<MIN_OBJ_DIST*MIN_OBJ_DIST) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					clusters.push_back(centers[i]);
					clusters.back().z = cluster_heights[i];
				}
			}
		}
		printf("Number of clusters: %d\n", clusters.size());
	}


//#define KMEANS_INIT

	void findClusters2(const PointCloud& cloud, vector<Point32>& clusters)
	{
		if (cloud.get_pts_size()==0) return;

#ifdef KMEANS_INIT
		// initialize clusters using kmeans
		const int NUM_CLUSTERS = 30;

		int count = cloud.get_pts_size();
		CvMat* points = cvCreateMat( count, 2, CV_32FC1 );
		CvMat* labels = cvCreateMat( count, 1, CV_32SC1 );
		CvMat* centers_ = cvCreateMat( NUM_CLUSTERS, 2, CV_32FC1 );

		for (int i=0;i<count;++i) {
			float* ptr = (float*)(points->data.ptr + i * points->step);
			ptr[0] = cloud.pts[i].x;
			ptr[1] = cloud.pts[i].y;
		}

		cvKMeans2(points, NUM_CLUSTERS, labels, cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.005 ),5,0,0,centers_);


		float step = CLUSTER_RADIUS;
		NNGridIndexer index(cloud);
//		vector<int> indices(cloud.get_pts_size());
		vector<Point32> centers(NUM_CLUSTERS);
		vector<Point32> means(NUM_CLUSTERS);
		Point32 p;
		double total_dist = 0;
		for (int i=0;i<NUM_CLUSTERS;++i) {
			float* ptr = (float*)(centers_->data.ptr + i * centers_->step);
			p.x = ptr[0];
			p.y = ptr[1];
			centers[i] = p;
			Point32 mean;
			int count = index.computeMean(p, step, mean);
			means[i]= mean;
			total_dist += dist2D(mean, p);
		}

		cvReleaseMat(&points);
		cvReleaseMat(&labels);
		cvReleaseMat(&centers_);

#else
		float xmin = cloud.pts[0].x;
		float xmax = cloud.pts[0].x;
		float ymin = cloud.pts[0].y;
		float ymax = cloud.pts[0].y;

		for (size_t i=1;i<cloud.get_pts_size();++i) {
			if (cloud.pts[i].x<xmin) xmin = cloud.pts[i].x;
			if (cloud.pts[i].x>xmax) xmax = cloud.pts[i].x;
			if (cloud.pts[i].y<ymin) ymin = cloud.pts[i].y;
			if (cloud.pts[i].y>ymax) ymax = cloud.pts[i].y;
		}

		float step = CLUSTER_RADIUS;

		NNGridIndexer index(cloud);
//		vector<int> indices(cloud.get_pts_size());

		// getting the initial centers
		vector<Point32> centers;
		vector<Point32> means;
		Point32 p;

		double total_dist = 0;
		for (double x = xmin;x<xmax;x+=step/2) {
			for (double y = ymin;y<ymax;y+=step/2) {

				p.x = x;
				p.y = y;

				Point32 mean;
				int found = index.computeMean(p, step, mean);

				if (found>10) {
					centers.push_back(p);
					means.push_back(mean);
					total_dist += dist2D(mean, p);
				}
			}
		}

#endif

		int iter = 0;
		// mean-shift
		bool odd = true;
		while (total_dist>0.001) {

			if (odd) {
				total_dist = meanShiftIteration2(cloud, index, means, centers, step);
			}
			else {
				total_dist = meanShiftIteration2(cloud, index, centers, means, step);
			}
			odd = !odd;
			iter++;

			if (iter>7) break;
		}

		filterClusters(cloud, centers, clusters);

		printf("Total dist: %f\n", total_dist);

		for (size_t i=0;i<clusters.size();++i) {
			showCluster(clusters[i], step, i, cloud.header.stamp);
		}

	}


	/**
	 * Projects 3D point into image plane
	 * @param cam_info Camera info containing camera projection matrix
	 * @param point The 3D point
	 * @return Projected point
	 */
	Point project3DPointIntoImage(const image_msgs::CamInfo& cam_info, PointStamped point)
	{
		PointStamped image_point;
		tf_.transformPoint(cam_info.header.frame_id, point, image_point);
		Point pp; // projected point

		pp.x = cam_info.P[0]*image_point.point.x+
				cam_info.P[1]*image_point.point.y+
				cam_info.P[2]*image_point.point.z+
				cam_info.P[3];
		pp.y = cam_info.P[4]*image_point.point.x+
				cam_info.P[5]*image_point.point.y+
				cam_info.P[6]*image_point.point.z+
				cam_info.P[7];
		pp.z = cam_info.P[8]*image_point.point.x+
				cam_info.P[9]*image_point.point.y+
				cam_info.P[10]*image_point.point.z+
				cam_info.P[11];

		pp.x /= pp.z;
		pp.y /= pp.z;
		pp.z = 1;

		return pp;
	}


	void getCameraIntrinsics(image_msgs::CamInfo& lcinfo)
	{
		lcinfo.header.frame_id = cloud->header.frame_id;
		lcinfo.header.stamp = cloud->header.stamp;

		lcinfo.P[0] = 725.00002432;
		lcinfo.P[1] = 0.0;
		lcinfo.P[2] = 321.35299836;
		lcinfo.P[3] = 0.0;
		lcinfo.P[4] = 0.0;
		lcinfo.P[5] = 725.00002432;
		lcinfo.P[6] = 210.43089442;
		lcinfo.P[7] = 0;
		lcinfo.P[8] = 0;
		lcinfo.P[9] = 0;
		lcinfo.P[10] = 1;
		lcinfo.P[11] = 0;
	}

	void projectClusters(const PointCloud& objects_table_frame, const vector<Point32>& clusters)
	{
		image_msgs::CamInfo lcinfo;
		getCameraIntrinsics(lcinfo);

		Point pp[8];
		for (size_t i=0;i<clusters.size();++i) {
			PointStamped ps;
			ps.header.frame_id = objects_table_frame.header.frame_id;
			ps.header.stamp = objects_table_frame.header.stamp;
			ps.point.z = 0;


			ps.point.x = clusters[i].x - CLUSTER_RADIUS;
			ps.point.y = clusters[i].y - CLUSTER_RADIUS;
			pp[0] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x - CLUSTER_RADIUS;
			ps.point.y = clusters[i].y + CLUSTER_RADIUS;
			pp[1] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x + CLUSTER_RADIUS;
			ps.point.y = clusters[i].y + CLUSTER_RADIUS;
			pp[2] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x + CLUSTER_RADIUS;
			ps.point.y = clusters[i].y - CLUSTER_RADIUS;
			pp[3] = project3DPointIntoImage(lcinfo, ps);

			ps.point.z = clusters[i].z;
			ps.point.x = clusters[i].x - CLUSTER_RADIUS;
			ps.point.y = clusters[i].y - CLUSTER_RADIUS;
			pp[4] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x - CLUSTER_RADIUS;
			ps.point.y = clusters[i].y + CLUSTER_RADIUS;
			pp[5] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x + CLUSTER_RADIUS;
			ps.point.y = clusters[i].y + CLUSTER_RADIUS;
			pp[6] = project3DPointIntoImage(lcinfo, ps);

			ps.point.x = clusters[i].x + CLUSTER_RADIUS;
			ps.point.y = clusters[i].y - CLUSTER_RADIUS;
			pp[7] = project3DPointIntoImage(lcinfo, ps);

//			cvLine(left, cvPoint(pp[0].x+0.5,pp[0].y+0.5), cvPoint(pp[1].x+0.5,pp[1].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[1].x+0.5,pp[1].y+0.5), cvPoint(pp[2].x+0.5,pp[2].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[2].x+0.5,pp[2].y+0.5), cvPoint(pp[3].x+0.5,pp[3].y+0.5), CV_RGB(0,255,0));
			cvLine(left, cvPoint(pp[3].x+0.5,pp[3].y+0.5), cvPoint(pp[0].x+0.5,pp[0].y+0.5), CV_RGB(0,255,0));

//			cvLine(left, cvPoint(pp[4].x+0.5,pp[4].y+0.5), cvPoint(pp[5].x+0.5,pp[5].y+0.5), CV_RGB(0,255,0));
			cvLine(left, cvPoint(pp[5].x+0.5,pp[5].y+0.5), cvPoint(pp[6].x+0.5,pp[6].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[6].x+0.5,pp[6].y+0.5), cvPoint(pp[7].x+0.5,pp[7].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[7].x+0.5,pp[7].y+0.5), cvPoint(pp[4].x+0.5,pp[4].y+0.5), CV_RGB(0,255,0));
//
//			cvLine(left, cvPoint(pp[0].x+0.5,pp[0].y+0.5), cvPoint(pp[4].x+0.5,pp[4].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[1].x+0.5,pp[1].y+0.5), cvPoint(pp[5].x+0.5,pp[5].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[2].x+0.5,pp[2].y+0.5), cvPoint(pp[6].x+0.5,pp[6].y+0.5), CV_RGB(0,255,0));
//			cvLine(left, cvPoint(pp[3].x+0.5,pp[3].y+0.5), cvPoint(pp[7].x+0.5,pp[7].y+0.5), CV_RGB(0,255,0));
			cvLine(left, cvPoint(pp[0].x+0.5,pp[0].y+0.5), cvPoint(pp[5].x+0.5,pp[5].y+0.5), CV_RGB(0,255,0));
			cvLine(left, cvPoint(pp[3].x+0.5,pp[3].y+0.5), cvPoint(pp[6].x+0.5,pp[6].y+0.5), CV_RGB(0,255,0));


			ps.point.x = clusters[i].x;
			ps.point.y = clusters[i].y;
			ps.point.z = clusters[i].z/2;

			Point center_pp = project3DPointIntoImage(lcinfo, ps);

			cvCircle(left, cvPoint(int(center_pp.x), int(center_pp.y)), 5, CV_RGB(0,255,0));

		}
	}


	void publishClusters(const PointCloud& cloud, const vector<Point32>& clusters)
	{
		PointCloud object;

		object.header.frame_id = cloud.header.frame_id;
		object.header.stamp = cloud.header.stamp;

		for (size_t k=0;k<clusters.size();++k) {
			object.pts.clear();
			for (size_t i=0;i<cloud.get_pts_size();++i) {
				if (dist2D(clusters[k],cloud.pts[i])< CLUSTER_RADIUS*CLUSTER_RADIUS ) {
					object.pts.push_back(cloud.pts[i]);
				}
			}
			object_pub_.publish(object);
		}
	}

	void findObjectPositionsFromStereo(vector<CvPoint>& locations, vector<float>& scales)
	{
		vector<double> plane;
		PointCloud objects_pc;
		PointCloud plane_pc;
		filterTablePlane(plane,objects_pc,plane_pc);

//		publish("~inliers", plane_pc);
//		publish("~outliers", objects_pc);


		PointCloud projected_objects = projectToPlane(objects_pc, plane);


		PointStamped table_point;
		table_point.header.frame_id = projected_objects.header.frame_id;
		table_point.header.stamp = projected_objects.header.stamp;
		table_point.point.x = projected_objects.pts[0].x;
		table_point.point.y = projected_objects.pts[0].y;
		table_point.point.z = projected_objects.pts[0].z;

		addTableFrame(table_point,plane);

		PointCloud objects_table_frame;
		tf_.transformPointCloud("table_frame", objects_pc, objects_table_frame);

		drawTableBBox(objects_table_frame);

		// find clusters

		vector<Point32> clusters;
		findClusters2(objects_table_frame, clusters);

		publishClusters(objects_table_frame,clusters);

		objects_pub_.publish(objects_table_frame);

		// reproject bboxes in image
//		projectClusters(objects_table_frame, clusters);

		getCameraIntrinsics(lcinfo);

		locations.resize(clusters.size());
		scales.resize(clusters.size());
		for (size_t i=0;i<clusters.size();++i) {
			PointStamped ps;
			ps.header.frame_id = objects_table_frame.header.frame_id;
			ps.header.stamp = objects_table_frame.header.stamp;

			// compute location
			ps.point.x = clusters[i].x;
			ps.point.y = clusters[i].y;
			ps.point.z = clusters[i].z/2;
			Point pp = project3DPointIntoImage(lcinfo, ps);

			locations[i].x = int(pp.x);
			locations[i].y = int(pp.y);

			// compute scale
			ps.point.z = 0;
			Point pp1 = project3DPointIntoImage(lcinfo, ps);
			ps.point.z = clusters[i].z;
			Point pp2 = project3DPointIntoImage(lcinfo, ps);

			float dist = sqrt(dist2D(pp1,pp2));
//			printf("Pixel height: %f\n", dist);
			printf("Real height: %f\n", clusters[i].z);
			scales[i] = dist/clusters[i].z;  // pixels per meter

			cvCircle(left, locations[i], 5, CV_RGB(0,255,0));

		}


	}



	void runRecognitionLambertian()
	{
		// acquire cv_mutex lock
		//        boost::unique_lock<boost::mutex> images_lock(cv_mutex);

		// goes to sleep until some images arrive
		//        images_ready.wait(images_lock);
		//        printf("Woke up, processing images\n");


		vector<CvPoint> positions;
		vector<float> scales;
		findObjectPositionsFromStereo(positions, scales);

		doChamferMatching(left, positions, scales);

	}





//	/**
//	* \brief Filters a cloud point, retains only points coming from a specific region in the disparity image
//	*
//	* @param rect Region in disparity image
//	* @return Filtered point cloud
//	*/
//	robot_msgs::PointCloud filterPointCloud(const PointCloud& cloud, const CvRect & rect)
//	{
//		robot_msgs::PointCloud result;
//		result.header.frame_id = cloud.header.frame_id;
//		result.header.stamp = cloud.header.stamp;
//		int xchan = -1;
//		int ychan = -1;
//		for(size_t i = 0;i < cloud.chan.size();++i){
//			if(cloud.chan[i].name == "x"){
//				xchan = i;
//			}
//			if(cloud.chan[i].name == "y"){
//				ychan = i;
//			}
//		}
//
//		if(xchan != -1 && ychan != -1){
//			for(size_t i = 0;i < cloud.pts.size();++i){
//				int x = (int)(cloud.chan[xchan].vals[i]);
//				int y = (int)(cloud.chan[ychan].vals[i]);
//				if(x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height){
//					result.pts.push_back(cloud.pts[i]);
//				}
//			}
//
//		}
//
//
//		return result;
//	}


//	/**
//	* Callback from topic synchronizer, timeout
//	* @param t
//	*/
//	void image_cb_timeout(ros::Time t)
//	{
//		if(limage.header.stamp != t) {
//			printf("Timed out waiting for left image\n");
//		}
//
//		if(dimage.header.stamp != t) {
//			printf("Timed out waiting for disparity image\n");
//		}
//
//		//        if(stinfo.header.stamp != t) {
//		//            printf("Timed out waiting for stereo info\n");
//		//        }
//
//		if(cloud_fetch.header.stamp != t) {
//			printf("Timed out waiting for point cloud\n");
//		}
//	}


//	/**
//	* Callback from topic synchronizer, images ready to be consumed
//	* @param t
//	*/
//	void image_cb_all(ros::Time t)
//	{
//		// obtain lock on vision data
//		boost::lock_guard<boost::mutex> lock(cv_mutex);
//
//		if(lbridge.fromImage(limage, "bgr")){
//			if(left != NULL)
//				cvReleaseImage(&left);
//
//			left = cvCloneImage(lbridge.toIpl());
//		}
//		if(rbridge.fromImage(rimage, "bgr")){
//			if(right != NULL)
//				cvReleaseImage(&right);
//
//			right = cvCloneImage(rbridge.toIpl());
//		}
//		if(dbridge.fromImage(dimage)){
//			if(disp != NULL)
//				cvReleaseImage(&disp);
//
//			//            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
//			disp = cvCloneImage(dbridge.toIpl());
//			//            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
//		}
//
//		cloud = cloud_fetch;
//
//		//        images_ready.notify_all();
//		runRecognitionLambertian();
//	}



public:
	/**
	* Needed for OpenCV event loop, to show images
	* @return
	*/
	bool spin()
	{
		while (nh_.ok())
		{
			int key = cvWaitKey(10)&0x00FF;
			if(key == 27) //ESC
				break;

			ros::spinOnce();
		}

		return true;
	}

	void triggerEdgeDetection()
	{
//		doChamferMatching(left);
	}
};

RecognitionLambertian* node;

void on_templates_no(int value)
{
	node->templates_no = value;
	node->triggerEdgeDetection();
}

void on_edges_high(int value)
{
	node->edges_high = value;
	node->triggerEdgeDetection();
}


int main(int argc, char **argv)
{
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

	ros::init(argc, argv, "recognition_lambertian");
	node = new RecognitionLambertian();
	node->spin();
//	ros::spin();

	delete node;

	return 0;
}

