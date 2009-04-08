//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

/**
@mainpage

@htmlinclude manifest.html

\author Marius Muja, Sachin Chitta

@b

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

#include <robot_msgs/Point32.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/VisualizationMarker.h>


#include "image_msgs/CvBridge.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"

#include "topic_synchronizer.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_line.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <angles/angles.h>

#include <sys/time.h>

using namespace std;
using namespace robot_msgs;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Comparison operator for a vector of vectors
inline bool compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

typedef vector< pair<CvPoint, CvPoint> > Lines;

class DoorStereo : public ros::Node
{
  public:

    tf::TransformListener *tf_;


	image_msgs::Image limage;
	image_msgs::Image dimage;
	image_msgs::StereoInfo stinfo;
	image_msgs::DisparityInfo dispinfo;
	image_msgs::CamInfo rcinfo;
	image_msgs::CvBridge lbridge;
	image_msgs::CvBridge rbridge;
	image_msgs::CvBridge dbridge;

	robot_msgs::PointCloud cloud_fetch;
	robot_msgs::PointCloud cloud_;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	TopicSynchronizer<DoorStereo> sync;

	double line_min_dist_;



	boost::mutex cv_mutex;
//	boost::condition_variable images_ready;

    /********** Parameters that need to be gotten from the param server *******/
    int sac_min_points_per_model_;
    double sac_distance_threshold_;
    double eps_angle_, frame_multiplier_;
    int sac_min_points_left_;
    double door_min_width_, door_max_width_;
    robot_msgs::Point32 axis_;
    bool display_;
    bool do_edge_filter_;

    DoorStereo() : ros::Node("door_stereo"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL), sync(this, &DoorStereo::image_cb_all, ros::Duration().fromSec(0.1), &DoorStereo::image_cb_timeout)
    {
      tf_ = new tf::TransformListener(*this);
//      param<std::string>("~p_door_msg_topic_", door_msg_topic_, "door_message");                              // 10 degrees

      param ("~p_sac_min_points_per_model", sac_min_points_per_model_, 50);  // 100 points at high resolution
      param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.01);     // 3 cm
      param ("~p_eps_angle_", eps_angle_, 2.0);                              // 10 degrees
      param ("~p_frame_multiplier_", frame_multiplier_,1.0);
      param ("~p_sac_min_points_left", sac_min_points_left_, 10);
      param ("~p_door_min_width", door_min_width_, 0.8);                    // minimum width of a door: 0.8m
      param ("~p_door_max_width", door_max_width_, 1.4);                    // maximum width of a door: 1.4m
      param ("~display", display_, true);
      param ("~line_min_dist", line_min_dist_, 2.0);
      param ("~do_edge_filter", do_edge_filter_, true);

      double tmp_axis;
      param ("~p_door_axis_x", tmp_axis, 0.0);                    // maximum width of a door: 1.4m
      axis_.x = tmp_axis;
      param ("~p_door_axis_y", tmp_axis, 0.0);                    // maximum width of a door: 1.4m
      axis_.y = tmp_axis;
      param ("~p_door_axis_z", tmp_axis, 1.0);                    // maximum width of a door: 1.4m
      axis_.z = tmp_axis;

      eps_angle_ = angles::from_degrees (eps_angle_);                    // convert to radians

      if (display_) {
    	  ROS_INFO("Displaying images\n");
    	  cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    	  //cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
//    	  cvNamedWindow("contours", CV_WINDOW_AUTOSIZE);
    	  cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
      }

      advertise<robot_msgs::VisualizationMarker>( "visualizationMarker", 0 );
      advertise<robot_msgs::PointCloud>( "filtered_cloud", 0 );

      subscribeStereoData();

    };

    ~DoorStereo()
    {
      unadvertise("visualizationMarker");
      unsubscribeStereoData();
    }



    void subscribeStereoData()
    {

    	sync.reset();
    	std::list<std::string> left_list;
    	left_list.push_back(std::string("stereo/left/image_rect_color"));
    	left_list.push_back(std::string("stereo/left/image_rect"));
    	sync.subscribe(left_list, limage, 1);

    	//		std::list<std::string> right_list;
    	//		right_list.push_back(std::string("stereo/right/image_rect_color"));
    	//		right_list.push_back(std::string("stereo/right/image_rect"));
    	//		sync.subscribe(right_list, rimage, 1);

    	sync.subscribe("stereo/disparity", dimage, 1);
    	sync.subscribe("stereo/stereo_info", stinfo, 1);
    	sync.subscribe("stereo/disparity_info", dispinfo, 1);
    	sync.subscribe("stereo/right/cam_info", rcinfo, 1);
    	sync.subscribe("stereo/cloud", cloud_fetch, 1);
    	sync.ready();
    	//        sleep(1);
    }

    void unsubscribeStereoData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/disparity");
        unsubscribe("stereo/stereo_info");
        unsubscribe("stereo/disparity_info");
        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
    }

    void fitDoorEdges(PointCloud& cloud)
    {
      ROS_INFO("Received a point cloud with %d points in frame: %s",(int) cloud.pts.size(),cloud.header.frame_id.c_str());
      if(cloud.pts.empty())
      {
        ROS_WARN("Received an empty point cloud");
        return;
      }
      if ((int)cloud.pts.size() < sac_min_points_per_model_)
        return;


      vector<int> indices;
      vector<int> possible_door_edge_points;
      indices.resize(cloud.pts.size());

      for(unsigned int i=0; i < cloud.pts.size(); i++)      //Use all the indices
      {
        indices[i] = i;
      }

      // Find the dominant lines
      vector<vector<int> > inliers;
      vector<vector<double> > coeff;

      vector<Point32> line_segment_min;
      vector<Point32> line_segment_max;

      fitSACLines(&cloud,indices,inliers,coeff,line_segment_min,line_segment_max);

      //Publish all the lines as visualization markers
      for(unsigned int i=0; i < inliers.size(); i++)
      {
        robot_msgs::VisualizationMarker marker;
        marker.header.frame_id = cloud.header.frame_id;
        marker.header.stamp = ros::Time((uint64_t)0ULL);
        marker.id = i;
        marker.type = robot_msgs::VisualizationMarker::LINE_STRIP;
        marker.action = robot_msgs::VisualizationMarker::ADD;
        marker.x = 0.0;
        marker.y = 0.0;
        marker.z = 0.0;
        marker.yaw = 0.0;
        marker.pitch = 0.0;
        marker.roll = 0.0;
        marker.xScale = 0.01;
        marker.yScale = 0.1;
        marker.zScale = 0.1;
        marker.alpha = 255;
        marker.r = 0;
        marker.g = 255;
        marker.b = 0;
        marker.set_points_size(2);

        marker.points[0].x = line_segment_min[i].x;
        marker.points[0].y = line_segment_min[i].y;
        marker.points[0].z = line_segment_min[i].z;

        marker.points[1].x = line_segment_max[i].x;
        marker.points[1].y = line_segment_max[i].y;
        marker.points[1].z = line_segment_max[i].z;

        publish( "visualizationMarker", marker );
      }
   }

    void fitSACLines(PointCloud *points, vector<int> indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff, vector<Point32> &line_segment_min, vector<Point32> &line_segment_max)
    {
      Point32 minP, maxP;

      vector<int> inliers_local;

      // Create and initialize the SAC model
       sample_consensus::SACModelOrientedLine *model = new sample_consensus::SACModelOrientedLine ();
       sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, sac_distance_threshold_);
       sac->setMaxIterations (100);
       model->setDataSet (points, indices);

       robot_msgs::Vector3Stamped axis_transformed;
       robot_msgs::Vector3Stamped axis_original;
       ROS_INFO("Original axis: %f %f %f",axis_.x,axis_.y,axis_.z);

       axis_original.vector.x = axis_.x;
       axis_original.vector.y = axis_.y;
       axis_original.vector.z = axis_.z;
       axis_original.header.frame_id = "base_link";
       axis_original.header.stamp = ros::Time(0.0);

       tf_->transformVector("stereo_link",axis_original,axis_transformed);

       Point32 axis_point_32;
       axis_point_32.x = axis_transformed.vector.x;
       axis_point_32.y = axis_transformed.vector.y;
       axis_point_32.z = axis_transformed.vector.z;

       ROS_INFO("Transformed axis: %f %f %f",axis_point_32.x,axis_point_32.y,axis_point_32.z);

       model->setAxis (axis_point_32);
       model->setEpsAngle (eps_angle_);

      // Now find the best fit line to this set of points and a corresponding set of inliers
      // Prepare enough space
      int number_remaining_points = indices.size();

      while(number_remaining_points > sac_min_points_left_)
      {
        if(sac->computeModel())
        {
          if((int) sac->getInliers().size() < sac_min_points_per_model_)
            break;
          inliers.push_back(sac->getInliers());
          vector<double> model_coefficients;
          sac->computeCoefficients (model_coefficients);
          coeff.push_back (model_coefficients);

          //Find the edges of the line segments
          cloud_geometry::statistics::getLargestDiagonalPoints(*points, inliers.back(), minP, maxP);
          line_segment_min.push_back(minP);
          line_segment_max.push_back(maxP);

          fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", (int)sac->getInliers ().size (), coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

          // Remove the current inliers in the model
          number_remaining_points = sac->removeInliers ();
        }
      }
    }




    /**
     * \brief Finds edges in an image
     * @param img
     */
    void findVerticalEdges(IplImage *img, Lines& lines_out )
    {
        IplImage *gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
        cvCvtColor(img, gray, CV_RGB2GRAY);
        cvCanny(gray, gray, 20, 40);
        CvMemStorage *storage = cvCreateMemStorage(0);
        CvSeq *lines = 0;
        lines = cvHoughLines2(gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 360, 100, 200, 100);
        for(int i = 0;i < lines->total;i++){
            CvPoint *line = (CvPoint*)cvGetSeqElem(lines, i);
            CvPoint p1 = line[0];
            CvPoint p2 = line[1];
            if(abs(p1.x - p2.x) > 0){
                float min_angle = 80;
                float slope = float(abs(p1.y - p2.y)) / abs(p1.x - p2.x);
                float min_slope = tan(CV_PI / 2 - min_angle * CV_PI / 180);
                if(slope < min_slope)
                    continue;

//                printf("slope: %f, min_slope: %f\n", slope, min_slope);
            }
            lines_out.push_back(make_pair(p1,p2));
            cvLine(img, p1, p2, CV_RGB(255, 0, 0), 2, CV_AA, 0);
        }

        cvReleaseImage(&gray);
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Transform a given point from its current frame to a given target frame
 * \param tf a pointer to a TransformListener object
 * \param target_frame the target frame to transform the point into
 * \param stamped_in the input point
 * \param stamped_out the output point
 */
    inline void
    transformPoint (tf::TransformListener *tf, const std::string &target_frame,
                    const tf::Stamped< robot_msgs::Point32 > &stamped_in, tf::Stamped< robot_msgs::Point32 > &stamped_out)
    {
      tf::Stamped<tf::Point> tmp;
      tmp.stamp_ = stamped_in.stamp_;
      tmp.frame_id_ = stamped_in.frame_id_;
      tmp[0] = stamped_in.x;
      tmp[1] = stamped_in.y;
      tmp[2] = stamped_in.z;

      tf->transformPoint (target_frame, tmp, tmp);

      stamped_out.stamp_ = tmp.stamp_;
      stamped_out.frame_id_ = tmp.frame_id_;
      stamped_out.x = tmp[0];
      stamped_out.y = tmp[1];
      stamped_out.z = tmp[2];
    }



    /**
     * Callback from topic synchronizer, timeout
     * @param t
     */
    void image_cb_timeout(ros::Time t)
    {
        if(limage.header.stamp != t) {
            printf("Timed out waiting for left image\n");
        }

        if(dimage.header.stamp != t) {
            printf("Timed out waiting for disparity image\n");
        }

        if(stinfo.header.stamp != t) {
            printf("Timed out waiting for stereo info\n");
        }

        if(cloud_fetch.header.stamp != t) {
        	printf("Timed out waiting for point cloud\n");
        }
    }



    void filterPointCloud(const PointCloud& in_cloud, Lines& lines, PointCloud& out)
    {

    	int n_lines = lines.size();
    	vector<double> vdx;
    	vector<double> vdy;
    	vector<double> vd;


    	out.header.frame_id = in_cloud.header.frame_id;
    	out.header.stamp = in_cloud.header.stamp;
    	out.chan.resize(in_cloud.chan.size());
    	for (size_t c = 0; c < in_cloud.get_chan_size();++c) {
    		out.chan[c].name = in_cloud.chan[c].name;
    	}


		int xchan = -1;
		int ychan = -1;

		for (size_t i=0;i<in_cloud.chan.size();++i) {
			if (in_cloud.chan[i].name == "x") {
				xchan = i;
			}
			if (in_cloud.chan[i].name == "y") {
				ychan = i;
			}
		}

		if (xchan!=-1 && ychan!=-1) {

			for (int i=0;i<n_lines;++i) {
				double dx = lines[i].second.x-lines[i].first.x;
				double dy = lines[i].second.y-lines[i].first.y;
				double dxy = sqrt(dx*dx+dy*dy);
				vdx.push_back(dx);
				vdy.push_back(dy);
				vd.push_back(dxy);
			}


			for (size_t i=0; i<in_cloud.get_pts_size(); ++i) {
				float x = in_cloud.chan[xchan].vals[i];
				float y = in_cloud.chan[xchan].vals[i];

				for (int j=0;j<n_lines;++j) {
					// compute distance to edge
					double d = fabs((lines[j].first.y-y)*vdx[j]-(lines[j].first.x-x)*vdy[j])/vd[j];

					if (d<line_min_dist_) {
						out.pts.push_back(in_cloud.pts[i]);
						for (size_t c = 0; c < in_cloud.get_chan_size();++c) {
							out.chan[c].vals.push_back(in_cloud.chan[c].vals[i]);
						}
					}
				}
			}
		}

    }


    /**
     *
     */
    void runDetector()
    {
    	Lines lines;
    	PointCloud filtered_cloud;
    	findVerticalEdges(left, lines);
    	if (do_edge_filter_) {
    		filterPointCloud(cloud_,lines,filtered_cloud);
    		fitDoorEdges(filtered_cloud);
    		publish("filtered_cloud", filtered_cloud);
    	}
    	else {
    		fitDoorEdges(cloud_);
    	}

    	if (display_) {
    		cvShowImage("left", left);
    		cvShowImage("disparity", disp);
    	}
    }

    /**
     * Callback from topic synchronizer, images ready to be consumed
     * @param t
     */
    void image_cb_all(ros::Time t)
    {
        // obtain lock on vision data
        boost::lock_guard<boost::mutex> lock(cv_mutex);

        if(lbridge.fromImage(limage, "bgr")){
            if(left != NULL)
                cvReleaseImage(&left);

            left = cvCloneImage(lbridge.toIpl());
        }
        if(dbridge.fromImage(dimage)){
            if(disp != NULL)
                cvReleaseImage(&disp);

            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
        }

        cloud_ = cloud_fetch;

        runDetector();

//        images_ready.notify_all();
    }




	bool spin()
	{
		while (ok())
		{
			cv_mutex.lock();
			int key = cvWaitKey(3)&0x00FF;
			if(key == 27) //ESC
				break;

			cv_mutex.unlock();
			usleep(10000);
		}

		return true;
	}








};


/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  DoorStereo p;


  p.spin ();
  return (0);
}
/* ]--- */

