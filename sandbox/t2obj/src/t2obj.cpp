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

// modified by Min Sun from Recognition_lambertian

#include "t2obj.h"
#include <sstream>
using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;

//extern double max_height;
//extern double min_height;
//extern double min_depth;
//extern double max_depth;
//extern double cluster_radius;

template<typename T>
static double dist2D(const T& a, const T& b)
{
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

    void filterByZBounds( const PointCloud& pc, double zmin, double zmax, PointCloud& filtered_pc, PointCloud& filtered_outside)
	{
		vector<int> indices_remove;
		for (size_t i = 0;i<pc.get_points_size();++i) {
			if (pc.points[i].z>zmax || pc.points[i].z<zmin) {
				indices_remove.push_back(i);
			}
		}
		cloud_geometry::getPointCloudOutside (pc, indices_remove, filtered_pc);
		cloud_geometry::getPointCloud(pc, indices_remove, filtered_outside);
	}

//    void clearFromImage(IplImage* disp_img, const PointCloud& pc)
//	{
//		int xchan = -1;
//		int ychan = -1;
//		for(size_t i = 0;i < pc.channels.size();++i){
//			if(pc.channels[i].name == "x"){
//				xchan = i;
//			}
//			if(pc.channels[i].name == "y"){
//				ychan = i;
//			}
//		}
//
//		if (xchan==-1 || ychan==-1) {
//			ROS_ERROR("Cannot find image coordinates in the point cloud");
//			return;
//		}
//
//		// remove plane points from disparity image
//		for (size_t i=0;i<pc.get_points_size();++i) {
//			int x = pc.channels[xchan].values[i];
//			int y = pc.channels[ychan].values[i];
//			//			printf("(%d,%d)\n", x, y);
//			CV_PIXEL(unsigned char, disp_img, x, y)[0] = 0;
//		}
//	}

    bool fitSACPlane(const PointCloud& points, const vector<int> &indices,  // input
			vector<int> &inliers, vector<double> &coeff,  // output
			double dist_thresh, int min_points_per_model, const Point32& orientation, double eps_angle)
	{
		// Create and initialize the SAC model
		//sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
		sample_consensus::SACModelOrientedPlane *model = new sample_consensus::SACModelOrientedPlane ();
		sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
		sac->setMaxIterations (100);
		model->setDataSet ((PointCloud*)&points, indices);
		model->setAxis(orientation);
		model->setEpsAngle(eps_angle);

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
			cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.points.at(inliers[0]), viewpoint);

			ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (), coeff[0], coeff[1], coeff[2], coeff[3]);
		}
		else {
			ROS_ERROR ("Could not compute a planar model for %d points.", indices.size());
			return false;
		}
		
		cout << "plane.x" << coeff[0] << " plane.y" << coeff[1] << " plane.z" << coeff[2] <<endl;
		delete sac;
		delete model;
		return true;
	}

    PointCloud projectToPlane(const PointCloud& objects, const vector<double>& coefficients)
	{
		// clear "under the table" points
		vector<int> object_indices(objects.points.size());
		for (size_t i=0;i<objects.get_points_size();++i) {
			object_indices[i] = i;
		}

		PointCloud object_projections;
		object_projections.header.stamp = objects.header.stamp;
		object_projections.header.frame_id = objects.header.frame_id;

		cloud_geometry::projections::pointsToPlane(objects, object_indices, object_projections, coefficients);

		return object_projections;

	}

    void filterTablePlane(const PointCloud& cloud, vector<double>& coefficients, PointCloud& object_cloud, PointCloud& plane_cloud,
        PointCloud& filtered_outside, const Point32& orientation, double eps_angle, double min_depth, double max_depth)
	{

//		disp_clone = cvCloneImage(disp);

		PointCloud filtered_cloud;
		//PointCloud filtered_outside;

		filterByZBounds(cloud, min_depth, max_depth , filtered_cloud, filtered_outside );

//		clearFromImage(disp, filtered_outside);

		vector<int> indices(filtered_cloud.get_points_size());
		for (size_t i = 0;i<filtered_cloud.get_points_size();++i) {
			indices[i] = i;
		}

		vector<int> inliers;
		double dist_thresh = 0.01; // in meters
		int min_points = 200;

		fitSACPlane(filtered_cloud, indices, inliers, coefficients, dist_thresh, min_points, orientation, eps_angle);

		cloud_geometry::getPointCloud(filtered_cloud, inliers, plane_cloud);
		cloud_geometry::getPointCloudOutside (filtered_cloud, inliers, object_cloud);

//		clearFromImage(disp, plane_cloud);
	}

    void addTableFrame(PointStamped origin, const vector<double>& plane, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_)
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

		broadcaster_.sendTransform(table_pose_frame);
	}

    float GetCameraHeight(const PointCloud& pc, tf::TransformListener& tf_){
        PointCloud camera_center;
        PointCloud camera_center_table_frame;
        camera_center.header = pc.header;
        camera_center.channels.resize(pc.channels.size ());
        camera_center.points.resize(1);
        for (unsigned int d = 0; d < camera_center.channels.size (); d++)
        {
            camera_center.channels[d].name = pc.channels[d].name;
            camera_center.channels[d].values.resize(1);
        }
        camera_center.points[0].x = 0;
        camera_center.points[0].y = 0;
        camera_center.points[0].z = 0;
        for (unsigned int d = 0; d < camera_center.channels.size (); d++)
        {
            camera_center.channels[d].values[0] = pc.channels[d].values[0];
        }
        tf_.transformPointCloud("table_frame", camera_center, camera_center_table_frame);

        return camera_center_table_frame.points[0].z;
    }

    float CalHorizontalLine( const PointCloud& cloud, vector<double> plane_coeff, const sensor_msgs::CameraInfo& rcinfo, tf::TransformListener& tf_){
        // calculate new plane_coeff which includes the origin pts
        plane_coeff[3] = 0;

        // calulcate a pts on the plane and in front of the camera
        PointStamped ps;
        ps.header.frame_id = cloud.header.frame_id;
		ps.header.stamp = cloud.header.stamp;

		// compute location
        ps.point.x = 0;
        ps.point.z = 0.7;
		ps.point.y = -(plane_coeff[2]/plane_coeff[1])*ps.point.z;
		Point pp = project3DPointIntoImage(rcinfo, ps, tf_);

        // get imaeg coordinate of the pts

        return pp.y;

    }

    void GetPointCloud2pointNoTF( PointCloud& cloud, cv::Vector<Point>& point2d, const sensor_msgs::CameraInfo& rcinfo){

        PointStamped ps;
        ps.header.frame_id = cloud.header.frame_id;
		ps.header.stamp = cloud.header.stamp;
        point2d.resize( cloud.points.size());
        for (unsigned int i =0; i < cloud.points.size(); i++){
            // compute location
            ps.point.x = cloud.points[i].x;
            ps.point.y = cloud.points[i].y;
            ps.point.z = cloud.points[i].z;
            point2d[i] = project3DPointIntoImageNoTF(rcinfo, ps);
        }

    }
    void GetPointCloud2point( PointCloud& cloud, cv::Vector<Point>& point2d, const sensor_msgs::CameraInfo& rcinfo, tf::TransformListener& tf_){

        PointStamped ps;
        ps.header.frame_id = cloud.header.frame_id;
		ps.header.stamp = cloud.header.stamp;
        point2d.resize( cloud.points.size());
        for (unsigned int i =0; i < cloud.points.size(); i++){
            // compute location
            ps.point.x = cloud.points[i].x;
            ps.point.y = cloud.points[i].y;
            ps.point.z = cloud.points[i].z;
            point2d[i] = project3DPointIntoImage(rcinfo, ps, tf_);
        }

    }

	class NNGridIndexer
	{
		float xmin, xmax, ymin, ymax;
		float xd,yd;

		int resolution;

		float* grid;

	public:
		NNGridIndexer(const PointCloud& cloud)
		{
			xmin = cloud.points[0].x;
			xmax = cloud.points[0].x;
			ymin = cloud.points[0].y;
			ymax = cloud.points[0].y;
			for (size_t i=1;i<cloud.get_points_size();++i) {
				if (cloud.points[i].x<xmin) xmin = cloud.points[i].x;
				if (cloud.points[i].x>xmax) xmax = cloud.points[i].x;
				if (cloud.points[i].y<ymin) ymin = cloud.points[i].y;
				if (cloud.points[i].y>ymax) ymax = cloud.points[i].y;
			}

			resolution = 600;
			xd = (xmax-xmin)/(resolution-1);
			yd = (ymax-ymin)/(resolution-1);

			grid = new float[resolution*resolution];
			memset(grid,0,resolution*resolution*sizeof(float));

			for (size_t i=0;i<cloud.get_points_size();++i) {
				Point32 p = cloud.points[i];

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

#if 0
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
			xmin = cloud.points[0].x;
			xmax = cloud.points[0].x;
			ymin = cloud.points[0].y;
			ymax = cloud.points[0].y;
			for (size_t i=1;i<cloud.get_points_size();++i) {
				if (cloud.points[i].x<xmin) xmin = cloud.points[i].x;
				if (cloud.points[i].x>xmax) xmax = cloud.points[i].x;
				if (cloud.points[i].y<ymin) ymin = cloud.points[i].y;
				if (cloud.points[i].y>ymax) ymax = cloud.points[i].y;
			}

			resolution = 600;
			xd = (xmax-xmin)/resolution;
			yd = (ymax-ymin)/resolution;

			grid = new float[resolution*resolution];
			memset(grid,0,resolution*resolution*sizeof(float));

			for (size_t i=0;i<cloud.get_points_size();++i) {
				Point32 p = cloud.points[i];

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

#endif
    	double meanShiftIteration( const PointCloud& pc, NNGridIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step)
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


//#define CLUSTER_RADIUS 0.15
//#define MIN_OBJ_HEIGHT 0.1
#define MAX_CLUSTERING_ITER 10
#define MIN_OBJ_DISTANCE 0.05
#define MIN_POINTS_PER_CLUSTER 10

	void filterClusters(const PointCloud& cloud, vector<Point32>& centers, vector<PointCloud>& clusters, double min_height, double cluster_radius)
	{
		// compute cluster heights (use to prune clusters)
		vector<double> cluster_heights(centers.size(), 0);
		// compute cluster heights
		for(size_t i=0;i<cloud.get_points_size();++i) {
			for (size_t j=0;j<centers.size();++j) {
				if (dist2D(cloud.points[i], centers[j])<cluster_radius*cluster_radius) {
					cluster_heights[j] = max(cluster_heights[j], (double)cloud.points[i].z);
				}
			}
		}


		// remove overlapping clusters
		vector<Point32> centers_pruned;
		for (size_t i=0;i<centers.size();++i) {
			if ( cluster_heights[i]> min_height) {

				bool duplicate = false;
				// check if duplicate cluster
				for (size_t j=0;j<centers_pruned.size();++j) {
					if (dist2D(centers_pruned[j],centers[i])<MIN_OBJ_DISTANCE*MIN_OBJ_DISTANCE) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					centers_pruned.push_back(centers[i]);
					centers_pruned.back().z = cluster_heights[i];
				}
			}
		}


		// compute clusters
		PointCloud object;
		object.header.frame_id = cloud.header.frame_id;
		object.header.stamp = cloud.header.stamp;

		for (size_t k=0;k<centers_pruned.size();++k) {
			object.points.clear();
			for (size_t i=0;i<cloud.get_points_size();++i) {
				if (dist2D(centers_pruned[k],cloud.points[i])< cluster_radius*cluster_radius ) {
					object.points.push_back(cloud.points[i]);
				}
			}
			clusters.push_back(object);
		}

		centers = centers_pruned;

		ROS_INFO("Number of clusters: %d\n", centers_pruned.size());
	}



	void findTabletopClusters(const PointCloud& cloud, vector<Point32>& centers, vector<PointCloud>& clusters, double min_height, double cluster_radius)
	{
		if (cloud.get_points_size()==0) return;

		// get x,y ranges
		float xmin = cloud.points[0].x;
		float xmax = cloud.points[0].x;
		float ymin = cloud.points[0].y;
		float ymax = cloud.points[0].y;

		for (size_t i=1;i<cloud.get_points_size();++i) {
			if (cloud.points[i].x<xmin) xmin = cloud.points[i].x;
			if (cloud.points[i].x>xmax) xmax = cloud.points[i].x;
			if (cloud.points[i].y<ymin) ymin = cloud.points[i].y;
			if (cloud.points[i].y>ymax) ymax = cloud.points[i].y;
		}
		cout << "obj_xmin" << xmin << " obj_xmax" << xmax << " obj_ymin" << ymin << " obj_ymax" << ymax <<endl;

		float step = cluster_radius;
		cout << "step" << step<<endl;

		NNGridIndexer index(cloud);
		if (true){

		// getting the initial centers
//		vector<Point32> centers;
		vector<Point32> means;
		Point32 p;

		// layout initial clusters in a grid
		double total_dist = 0;
		for (double x = xmin;x<xmax;x+=step/2) {
			for (double y = ymin;y<ymax;y+=step/2) {
				p.x = x;
				p.y = y;

				Point32 mean;
				int found = index.computeMean(p, step, mean);

				if (found>MIN_POINTS_PER_CLUSTER) {
					centers.push_back(p);
					means.push_back(mean);
					total_dist += dist2D(mean, p);
				}
			}
		}
		cout << "initial number cluster=" << centers.size() <<endl;

		int iter = 0;
		// mean-shift
		bool odd = true;
		while (total_dist>0.001) {

			if (odd) {
				total_dist = meanShiftIteration(cloud, index, means, centers, step);
			}
			else {
				total_dist = meanShiftIteration(cloud, index, centers, means, step);
			}
			odd = !odd;
			iter++;

			if (iter>MAX_CLUSTERING_ITER) break;
		}
		cout << "number cluster after meanshift=" << centers.size() <<endl;

		filterClusters(cloud, centers, clusters, min_height, cluster_radius);
		cout << "number cluster after filter=" << centers.size() <<endl;
		}




//		printf("Total dist: %f\n", total_dist);

//		for (size_t i=0;i<clusters.size();++i) {
//			showCluster(clusters[i], step, i, cloud.header.stamp);
//		}

	}
	
    struct DistCloud
    {
        DistCloud(double _eps) : eps(_eps) {}
        inline bool operator()(const Point32& p1, const Point32& p2) const
        {
            double dist = (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
            return dist<=eps*eps;
        }
        double eps;
    };

    void Overlay2dpoint(cv::Mat& img, cv::Vector<Point>& pp, cv::Scalar Color){
        for (unsigned int i = 0; i< pp.size(); i++){
            cv::Point pp_tmp;
            pp_tmp.x = pp[i].x;
            pp_tmp.y = pp[i].y;
            circle( img, pp_tmp, 1, Color, 1);
        }
    }

    void findObjectPositionsFromStereo( const PointCloud& cloud, PointCloud& locations,
        vector<float>& scales, vector<Point> top, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_,
        const sensor_msgs::CameraInfo& rcinfo, cv::Mat& right, const Point32& orientation, double eps_angle, int indx, double max_height, double min_height,double min_depth, double max_depth, double cluster_radius)
	{       
                cout << "in findObjectPositionsFromStereo" <<endl;
		vector<double> plane;
		PointCloud tmp_objects_pc;
		PointCloud plane_pc;
		PointCloud outside_pc; //either too close or too far
		cout << "begin filterTablePlane" <<endl;
		filterTablePlane(cloud, plane, tmp_objects_pc,plane_pc, outside_pc, orientation, eps_angle, min_depth, max_depth);// fiting table plane
		cout << "finish filterTablePlane" <<endl;

		cout << "begin projectToPlane" <<endl;
		PointCloud projected_objects = projectToPlane(tmp_objects_pc, plane);
		cout << "finish projectToPlane" <<endl;

		if ( projected_objects.points.size() == 0)
			return;


		PointStamped table_point;
		table_point.header.frame_id = projected_objects.header.frame_id;
		table_point.header.stamp = projected_objects.header.stamp;
		table_point.point.x = projected_objects.points[0].x;
		table_point.point.y = projected_objects.points[0].y;
		table_point.point.z = projected_objects.points[0].z;

		cout << "begin add addTableFrame" <<endl;
		addTableFrame(table_point,plane, tf_, broadcaster_);
		cout << "finish add addTableFrame" <<endl;

		cout << "begin transformPointCloud" <<endl;
		PointCloud tmp_objects_table_frame;
		tf_.transformPointCloud("table_frame", tmp_objects_pc, tmp_objects_table_frame);
		cout << "finish transformPointCloud" <<endl;
		PointCloud plane_table_frame;
		tf_.transformPointCloud("table_frame", plane_pc, plane_table_frame);
		//get x and y range of the plane
		if (plane_table_frame.points.size() == 0)
			return;

		float plane_x_max = plane_table_frame.points[0].x;
		float plane_x_min = plane_table_frame.points[0].x;
		float plane_y_max = plane_table_frame.points[0].y;
		float plane_y_min = plane_table_frame.points[0].y;
		for (unsigned int i =0; i < plane_table_frame.points.size(); i++){
			if (plane_table_frame.points[i].x<plane_x_min) plane_x_min = plane_table_frame.points[i].x;
			if (plane_table_frame.points[i].x>plane_x_max) plane_x_max = plane_table_frame.points[i].x;
			if (plane_table_frame.points[i].y<plane_y_min) plane_y_min = plane_table_frame.points[i].y;
			if (plane_table_frame.points[i].y>plane_y_max) plane_y_max = plane_table_frame.points[i].y;
		}
		cout << "xmax" << plane_x_max << " xmin" << plane_x_min << " ymax" << plane_y_max << " ymin" << plane_y_min <<endl;
		cout << "begin finding outliners" <<endl;
		vector<int> InlinerInd;
		for (unsigned int i =0; i < tmp_objects_table_frame.points.size(); i++){
			if ( tmp_objects_table_frame.points[i].z >= min_height && tmp_objects_table_frame.points[i].z <= max_height &&
			    tmp_objects_table_frame.points[i].x >= (plane_x_min) && tmp_objects_table_frame.points[i].x <= (plane_x_max) &&
			    tmp_objects_table_frame.points[i].y >= (plane_y_min) && tmp_objects_table_frame.points[i].y <= (plane_y_max-0.1))
				InlinerInd.push_back(i);
		}
		cout << "finish finding outliners" <<endl;
		cout << "begin pruneing" <<endl;
		PointCloud objects_table_frame;
		PointCloud objects_pc;
		objects_table_frame.header.frame_id = tmp_objects_table_frame.header.frame_id;
		objects_table_frame.header.stamp = tmp_objects_table_frame.header.stamp;
		objects_table_frame.points.clear();
		objects_pc.header.frame_id = tmp_objects_pc.header.frame_id;
		objects_pc.header.stamp = tmp_objects_pc.header.stamp;
		objects_pc.points.clear();
		for (unsigned int i =0; i <InlinerInd.size(); i++){
			objects_pc.points.push_back( tmp_objects_pc.points[InlinerInd[i]]);
	//		objects_pc.channels.erase( objects_pc.channels.begin()+InlinerInd[i]);
			objects_table_frame.points.push_back( tmp_objects_table_frame.points[InlinerInd[i]]);
	//		objects_table_frame.channels.erase( objects_table_frame.channels.begin()+InlinerInd[i]);
		}
		cout << "finish pruneing" <<endl;

        	// transfer all clouds into point on images
		//cout << "begin could2point" <<endl;
                //cv::Vector<Point> objects_pp_w;
                //GetPointCloud2point( objects_pc, objects_pp_w, w_rcinfo, tf_);
		//cout << "finish could2point" <<endl;
        	//GetPointCloud2point( plane_pc, plane_pp, rcinfo, tf_);
        	//GetPointCloud2point( outside_pc, outside_pp, rcinfo, tf_);

		// debug plot objects_pp
		cout << "begin plot" <<endl;
		cv::namedWindow("right", 1);
		//for (unsigned int i =0; i < objects_pp_w.size(); i++){
		//    cv::circle( right, cv::Point( objects_pp_w[i].x, objects_pp_w[i].y), 1, cv::Scalar(255,0,0), 1);
		//}
		//for (unsigned int i =0; i < plane_pp.size(); i++){
		//    cv::circle( right, cv::Point( plane_pp[i].x, plane_pp[i].y), 1, cv::Scalar(0,255,0), 1);
		//}

		// find clusters
		vector<Point32> clusters;
		vector<PointCloud> clouds;
		findTabletopClusters(objects_table_frame, clusters, clouds, min_height, cluster_radius);
		cout << "num_cluster" << clusters.size()<<endl;
                
                //PointCloud tmp_locations;
		locations.header.frame_id = objects_table_frame.header.frame_id;
		locations.header.stamp = objects_table_frame.header.stamp;
                locations.points.resize(clusters.size());
		scales.resize(clusters.size());
                top.resize( clusters.size());
		for (size_t i=0;i<clusters.size();++i) {
			PointStamped ps;
			ps.header.frame_id = objects_table_frame.header.frame_id;
			ps.header.stamp = objects_table_frame.header.stamp;


			// compute location
			ps.point.x = clusters[i].x;
			ps.point.y = clusters[i].y;
			ps.point.z = clusters[i].z;
			Point pp = project3DPointIntoImage(rcinfo, ps, tf_);

			// compute scale
			ps.point.z = 0;
			Point bottom = project3DPointIntoImage(rcinfo, ps, tf_);  // compute obj bottom in image coordinate
			ps.point.z = clusters[i].z;
			top.push_back(project3DPointIntoImage(rcinfo, ps, tf_));

			float dist = sqrt(dist2D(bottom,top.back()));
			printf("Pixel height: %f\n", dist);
			printf("Real height: %f\n", clusters[i].z);
			scales[i] = dist/clusters[i].z;  // pixels per meter
			printf("Scale: %f\n", scales[i]);

//			cv::circle( right, cv::Point(bottom.x,bottom.y), 10, CV_RGB(0,255,0));
			cv::circle( right, cv::Point(top.back().x,top.back().y), 3, CV_RGB(0,255,0));
			std::ostringstream oss;
			oss<<indx+i;
			cv::putText( right, oss.str().c_str(), cv::Point(top.back().x,top.back().y), 2, 2., CV_RGB(0,255,0));

                        clusters[i].z = 0;
                        locations.points[i] = clusters[i];

		}
		cv::imshow("right", right);
		cv::waitKey(100);
		cout << "finished plot" <<endl;

                // transform the table frame back to stereo frame
                //tf_.transformPointCloud(objects_pc.header.frame_id, tmp_locations, locations);
                
	}

    Point project3DPointIntoImageNoTF(const sensor_msgs::CameraInfo& cam_info, PointStamped image_point)
	{
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

    Point project3DPointIntoImage(const sensor_msgs::CameraInfo& cam_info, PointStamped point, tf::TransformListener& tf_)
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
