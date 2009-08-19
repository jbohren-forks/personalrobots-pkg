/********************************************************************
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

// Author: Min Sun
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_synchronizer2/topic_synchronizer.h>
#include <sstream>
#include <string>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/projections.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Empty.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace std;

class texture_on_off
{

public:
    ros::NodeHandle n;

    Mat right;

    tf::TransformListener tf_;
    tf::TransformBroadcaster broadcaster_;
    TopicSynchronizer sync_;
    sensor_msgs::ImageConstPtr rimage;
    sensor_msgs::CvBridge rbridge;
    sensor_msgs::CameraInfoConstPtr rcinfo_;

    ros::Subscriber right_image_sub_;
    ros::Subscriber right_caminfo_image_sub_;
    ros::Subscriber cloud_sub_;

    ros::Publisher get_params_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher image_pub_;

    sensor_msgs::PointCloudConstPtr cloud;

    int image_sequence;
    int off_exposure;
    int on_exposure;

    texture_on_off(): sync_(&texture_on_off::syncCallback, this)
    {
        // get parameters
        n.param( "~image_sequence", image_sequence, 1);
	cout << "image_sequence," << image_sequence <<endl;
        n.param( "~off_exposure", off_exposure, 1);
	cout << "off_exposure," << off_exposure <<endl;
        n.param( "~on_exposure", on_exposure, 1);
	cout << "on_exposure," << on_exposure <<endl;
        cout << "Finish handle input" << endl;

        // subscribe to topics
        right_image_sub_ = n.subscribe("narrow_stereo/right/image_rect", 1, sync_.synchronize(&texture_on_off::rightImageCallback, this));
        right_caminfo_image_sub_ = n.subscribe("narrow_stereo/right/cam_info", 1, sync_.synchronize(&texture_on_off::rightCameraInfoCallback, this));
        cloud_sub_ = n.subscribe( "narrow_stereo/cloud", 1, sync_.synchronize(&texture_on_off::cloudCallback, this) );

        get_params_pub_ = n.advertise<std_msgs::Empty>("/narrow_stereo/stereodcam/check_params",1);
        cloud_pub_ = n.advertise<sensor_msgs::PointCloud>("~/cloud",1);
        image_pub_ = n.advertise<sensor_msgs::Image>("~/image",1);

	texture_light_on_flag = false;
	image_sequence_count = 0;

	cv::namedWindow("imgshow", 1);
    }

    bool spin()
    {
            while (n.ok())
            {
                    ros::spinOnce();
            }
            return true;
    }

private:

    bool texture_light_on_flag;
    int image_sequence_count;
	
    void rightImageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
            rimage = image;
            if(rbridge.fromImage(*rimage, "bgr8")) {
                cout << " right image call back"<<endl;
	    	right = Mat(rbridge.toIpl(), false);
            }
    }

    void rightCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
	{
            cout << " right camera info call back"<<endl;
	    rcinfo_ = info;
	}

    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
	{
                cout << " point cloud call back"<<endl;
		cloud = point_cloud;
	}

    void syncCallback()
    {
        cout << "syncCallback" << endl;
	image_sequence_count++;
	if (image_sequence_count >= image_sequence){
		if (texture_light_on_flag){
			cout << "led on" << endl;
			system("pwd");
			system("./led_ctrl eth2 on");
			texture_light_on_flag = false;
			image_sequence_count = 0;
			n.setParam("/narrow_stereo/stereodcam/exposure", 
				on_exposure);
		}else{
			cout << "led off" << endl;
			system("pwd");
			system("./led_ctrl eth2 off");
			texture_light_on_flag = true;
			image_sequence_count = 0;
			n.setParam("/narrow_stereo/stereodcam/exposure", 
				off_exposure);
		}
		std_msgs::Empty tmp;
            	get_params_pub_.publish(tmp);
	}else if( image_sequence_count == (int)(image_sequence/2)){
		if (texture_light_on_flag){
			cout << "publish image" << endl;
			image_pub_.publish(rimage);
		}else{
			cout << "publish cloud" << endl;
			cloud_pub_.publish(cloud);
		}
	}

	// show image
	cv::imshow("imgshow", right);
	cv::waitKey(100);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "texture_on_off");
    texture_on_off node;

    if (system(NULL)) puts ("Ok");
    else exit (1);

    node.spin();
    return true;
}
