
#include "ObjectInPerspective.h"
using namespace std;
using namespace robot_msgs;

template<typename T>
static double dist2D(const T& a, const T& b)
{
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

    void filterByZBounds( const PointCloud& pc, double zmin, double zmax, PointCloud& filtered_pc, PointCloud& filtered_outside)
	{
		vector<int> indices_remove;
		for (size_t i = 0;i<pc.get_pts_size();++i) {
			if (pc.pts[i].z>zmax || pc.pts[i].z<zmin) {
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
//		for(size_t i = 0;i < pc.chan.size();++i){
//			if(pc.chan[i].name == "x"){
//				xchan = i;
//			}
//			if(pc.chan[i].name == "y"){
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
//		for (size_t i=0;i<pc.get_pts_size();++i) {
//			int x = pc.chan[xchan].vals[i];
//			int y = pc.chan[ychan].vals[i];
//			//			printf("(%d,%d)\n", x, y);
//			CV_PIXEL(unsigned char, disp_img, x, y)[0] = 0;
//		}
//	}

    bool fitSACPlane(const PointCloud& points, const vector<int> &indices,  // input
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

    PointCloud projectToPlane(const PointCloud& objects, const vector<double>& coefficients)
	{
		// clear "under the table" points
		vector<int> object_indices(objects.pts.size());
		for (size_t i=0;i<objects.get_pts_size();++i) {
			object_indices[i] = i;
		}

		PointCloud object_projections;
		object_projections.header.stamp = objects.header.stamp;
		object_projections.header.frame_id = objects.header.frame_id;

		cloud_geometry::projections::pointsToPlane(objects, object_indices, object_projections, coefficients);

		return object_projections;

	}

    void filterTablePlane(const PointCloud& cloud, vector<double>& coefficients, PointCloud& object_cloud, PointCloud& plane_cloud)
	{

//		disp_clone = cvCloneImage(disp);

		PointCloud filtered_cloud;
		PointCloud filtered_outside;

		filterByZBounds(cloud,0.1, 1.5 , filtered_cloud, filtered_outside );

//		clearFromImage(disp, filtered_outside);

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
        camera_center.chan.resize(pc.chan.size ());
        camera_center.pts.resize(1);
        for (unsigned int d = 0; d < camera_center.chan.size (); d++)
        {
            camera_center.chan[d].name = pc.chan[d].name;
            camera_center.chan[d].vals.resize(1);
        }
        camera_center.pts[0].x = 0;
        camera_center.pts[0].y = 0;
        camera_center.pts[0].z = 0;
        for (unsigned int d = 0; d < camera_center.chan.size (); d++)
        {
            camera_center.chan[d].vals[0] = pc.chan[d].vals[0];
        }
        tf_.transformPointCloud("table_frame", camera_center, camera_center_table_frame);

        return camera_center_table_frame.pts[0].z;
    }

    float CalHorizontalLine( const PointCloud& cloud, vector<double> plane_coeff, const image_msgs::CamInfo& lcinfo, tf::TransformListener& tf_){
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
		Point pp = project3DPointIntoImage(lcinfo, ps, tf_);

        // get imaeg coordinate of the pts

        return pp.y;

    }

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

#define CLUSTER_RADIUS 0.15/2
#define MIN_OBJ_HEIGHT 0.06
#define MIN_OBJ_DIST 0.05
    double meanShiftIteration(const PointCloud& pc, NNGridIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step)
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

	void filterClusters(const PointCloud& cloud, vector<Point32>& centers, vector<Point32>& clusters)
	{
		vector<double> cluster_heights(centers.size(), 0);
//		vector<double> cluster_center_depth(centers.size(), 0);

		printf("Number of centers: %d\n", centers.size());

		// compute cluster heights
		for(size_t i=0;i<cloud.get_pts_size();++i) {
			for (size_t j=0;j<centers.size();++j) {
				if (dist2D(cloud.pts[i], centers[j])<CLUSTER_RADIUS*CLUSTER_RADIUS) {
					cluster_heights[j] = max(cluster_heights[j], (double)cloud.pts[i].z);
//					cluster_center_depth[j] = cluster_center_depth[j]+;
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
				total_dist = meanShiftIteration(cloud, index, means, centers, step);
			}
			else {
				total_dist = meanShiftIteration(cloud, index, centers, means, step);
			}
			odd = !odd;
			iter++;

			if (iter>7) break;
		}

		filterClusters(cloud, centers, clusters);

		printf("Total dist: %f\n", total_dist);

//		for (size_t i=0;i<clusters.size();++i) {
//			showCluster(clusters[i], step, i, cloud.header.stamp);
//		}

	}

    float findObjectPositionsFromStereo(const PointCloud& cloud, vector<CvPoint>& locations, vector<CvPoint>& obj_bottom,
        vector<float>& scales, vector<float>& scales_msun, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_,
        const image_msgs::CamInfo& lcinfo)
	{
		vector<double> plane;
		PointCloud objects_pc;
		PointCloud plane_pc;
		filterTablePlane(cloud, plane,objects_pc,plane_pc);// fiting table plane

		PointCloud projected_objects = projectToPlane(objects_pc, plane);


		PointStamped table_point;
		table_point.header.frame_id = projected_objects.header.frame_id;
		table_point.header.stamp = projected_objects.header.stamp;
		table_point.point.x = projected_objects.pts[0].x;
		table_point.point.y = projected_objects.pts[0].y;
		table_point.point.z = projected_objects.pts[0].z;

		addTableFrame(table_point,plane, tf_, broadcaster_);

		PointCloud objects_table_frame;
		tf_.transformPointCloud("table_frame", objects_pc, objects_table_frame);

        // getting camera height
        float camera_height = GetCameraHeight(cloud, tf_);
//        cout << "camera_height " << camera_height << endl;

        float horizontal_line_row = CalHorizontalLine( cloud, plane, lcinfo, tf_);
//        cout << "horizontal_line_row " << horizontal_line_row << endl;

		// find clusters
		vector<Point32> clusters;
		findClusters2(objects_table_frame, clusters);

		locations.resize(clusters.size());
		obj_bottom.resize(clusters.size());
		scales.resize(clusters.size());
		scales_msun.resize(clusters.size());
		for (size_t i=0;i<clusters.size();++i) {
			PointStamped ps;
			ps.header.frame_id = objects_table_frame.header.frame_id;
			ps.header.stamp = objects_table_frame.header.stamp;

			// compute location
			ps.point.x = clusters[i].x;
			ps.point.y = clusters[i].y;
			ps.point.z = clusters[i].z/2;
			Point pp = project3DPointIntoImage(lcinfo, ps, tf_);

			locations[i].x = int(pp.x);
			locations[i].y = int(pp.y);

			// compute scale
			ps.point.z = 0;
			Point pp1 = project3DPointIntoImage(lcinfo, ps, tf_);  // compute obj bottom in image coordinate
			ps.point.z = clusters[i].z;
			Point pp2 = project3DPointIntoImage(lcinfo, ps, tf_);

			float dist = sqrt(dist2D(pp1,pp2));
			printf("Pixel height: %f\n", dist);
			printf("Real height: %f\n", clusters[i].z);
			scales[i] = dist/clusters[i].z;  // pixels per meter
			printf("Scale: %f\n", scales[i]);

            //cout << "obj_bottom " << pp1.y << endl;
            obj_bottom[i].x = pp1.x;
            obj_bottom[i].y = pp1.y;

            scales_msun[i] = (pp1.y - horizontal_line_row)/camera_height; // pixels per meter
            printf("Scale_msun: %f\n", scales_msun[i]);
//			cvCircle(left, locations[i], 5, CV_RGB(0,255,0));

		}
		return camera_height;
	}

    Point project3DPointIntoImage(const image_msgs::CamInfo& cam_info, PointStamped point, tf::TransformListener& tf_)
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
