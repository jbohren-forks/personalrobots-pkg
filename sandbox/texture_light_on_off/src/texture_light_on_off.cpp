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

#include "stereo_msgs/DisparityInfo.h"
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

    IplImage* right;
    IplImage* right_clone;
    IplImage* disp;

    tf::TransformListener tf_;
    tf::TransformBroadcaster broadcaster_;
    TopicSynchronizer sync_;
    sensor_msgs::ImageConstPtr rimage;
    sensor_msgs::ImageConstPtr dimage;
    stereo_msgs::DisparityInfoConstPtr dispinfo;
    sensor_msgs::CvBridge rbridge;
    sensor_msgs::CvBridge dbridge;
    sensor_msgs::CameraInfoConstPtr rcinfo_;

    ros::Subscriber right_image_sub_;
    ros::Subscriber right_caminfo_image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber disparity_sub_;
    ros::Subscriber dispinfo_sub_;

    ros::Publisher get_params_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher image_pub_;
    ros::Publisher disp_pub_;

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
        right_image_sub_ = n.subscribe("narrow_stereo/left/image_rect", 1, sync_.synchronize(&texture_on_off::rightImageCallback, this));
        right_caminfo_image_sub_ = n.subscribe("narrow_stereo/left/cam_info", 1, sync_.synchronize(&texture_on_off::rightCameraInfoCallback, this));
        cloud_sub_ = n.subscribe( "narrow_stereo/cloud", 1, sync_.synchronize(&texture_on_off::cloudCallback, this) );
	disparity_sub_ = n.subscribe("narrow_stereo/disparity", 1, sync_.synchronize(&texture_on_off::disparityImageCallback, this));
	dispinfo_sub_ = n.subscribe("narrow_stereo/disparity_info", 1, sync_.synchronize(&texture_on_off::dispinfoCallback, this));

        get_params_pub_ = n.advertise<std_msgs::Empty>("/narrow_stereo/stereodcam/check_params",1);
        cloud_pub_ = n.advertise<sensor_msgs::PointCloud>("~/cloud",1);
        image_pub_ = n.advertise<sensor_msgs::Image>("~/image",1);
        disp_pub_ = n.advertise<sensor_msgs::Image>("~/disp",1);

	texture_light_on_flag = false;
	image_sequence_count = 0;

	cv::namedWindow("imgshow", 1);
	right_clone = NULL;
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
            cout << " right image call back"<<endl;
            rimage = image;
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
    void disparityImageCallback(const sensor_msgs::Image::ConstPtr& image)
	{
		dimage = image;
	}
    void dispinfoCallback(const stereo_msgs::DisparityInfo::ConstPtr& dinfo)
	{
		dispinfo = dinfo;
	}

    void syncCallback()
    {
        cout << "syncCallback" << endl;
	image_sequence_count++;
	if (image_sequence_count >= image_sequence){
		if (texture_light_on_flag){
			cout << "led on" << endl;
			system("pwd");
			system("../ros-pkg/sandbox/texture_light_on_off/led_ctrl eth2 on");
			texture_light_on_flag = false;
			image_sequence_count = 0;
			n.setParam("/narrow_stereo/stereodcam/exposure", 
				on_exposure);
		}else{
			cout << "led off" << endl;
			system("pwd");
			system("../ros-pkg/sandbox/texture_light_on_off/led_ctrl eth2 off");
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
		    	if(rbridge.fromImage(*rimage, "bgr8")) {
				right = rbridge.toIpl();
				right_clone = cvCloneImage(right);
		    	}
		}else{
			cout << "publish cloud" << endl;
			if (right_clone!=NULL){
				cloud_pub_.publish(cloud);
				disp_pub_.publish(dimage);
			if(dbridge.fromImage(*dimage) ) {
				disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
				cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo->dpp);
				cout << "disp.width"<<disp->width<<" height"<<disp->height<<endl;
				cout << "right.width"<< right_clone->width <<" height"<< right_clone->height <<" nCh"<<right_clone->nChannels<<endl;
				
				for (unsigned int y=0; y<right_clone->height;y++){
					char* dispPtr = (char*)(disp->imageData + y*disp->widthStep);;
					char* imagePtr = (char*)(right_clone->imageData + y*right_clone->widthStep);;
					for (unsigned int x=0; x<right_clone->width;x++){
						if ( dispPtr[x] == 0)
							for (unsigned int chid=0; chid< right_clone->nChannels; chid++){
								imagePtr[right_clone->nChannels*x+chid] = 0;
							}
					}
				}				
				cv::imshow("imgshow", right_clone);
				cv::waitKey(100);	
			}
			}
		}
	}


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
