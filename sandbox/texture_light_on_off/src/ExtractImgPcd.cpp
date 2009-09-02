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

// Author: Min Sun
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"

#include <point_cloud_mapping/cloud_io.h>

#include <std_msgs/String.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"

using namespace cv;
using namespace std;

class ExtractImgPcd
{

public:
    ros::NodeHandle n;

    Mat image_clone;

    sensor_msgs::PointCloud pc;

    sensor_msgs::ImageConstPtr image_;
    sensor_msgs::CvBridge bridge_;
    sensor_msgs::CameraInfoConstPtr cinfo_;
    sensor_msgs::PointCloudConstPtr cloud_;

    ros::Subscriber image_sub_;
    ros::Subscriber caminfo_image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber bag_name_sub_;

    string bag_filename;
    string img_folder;
    string pcd_folder;

    ExtractImgPcd()
    {
        // get parameters
        string tmp_String = "./";
        n.param( "~img_folder", img_folder, tmp_String);
	cout << "img_folder" << img_folder <<endl;
        n.param( "~pcd_folder", pcd_folder, tmp_String);
	cout << "pcd_folder" << pcd_folder <<endl;

        // subscribe to topics
        image_sub_ = n.subscribe("t_on_off/image", 1, &ExtractImgPcd::ImageCallback, this);
        caminfo_image_sub_ = n.subscribe("t_on_off/caminfo", 1, &ExtractImgPcd::CameraInfoCallback, this);
        cloud_sub_ = n.subscribe( "t_on_off/dense_cloud", 1, &ExtractImgPcd::cloudCallback, this );
        bag_name_sub_ = n.subscribe( "bag_name", 1, &ExtractImgPcd::bagNameCallback, this );

    }

    bool spin()
    {
        while (n.ok())
        {
                ros::spinOnce();
		// evoke reoplay
        }
        return true;
    }

private:

	void bagNameCallback(const std_msgs::StringConstPtr& msg_ptr)
	{
		bag_filename= msg_ptr->data;
		cout << "bag_filename" << bag_filename << endl;
	}

    void ImageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        image_ = image;
        if( bridge_.fromImage(*image_, "bgr8")) {
            cout << " image call back"<<endl;
            image_clone = Mat(bridge_.toIpl(), false);
	    // svae image
	    string img_tmp = img_folder+"/"+bag_filename+".png";
	    cout << "imgfilename" << img_tmp << endl;
	    cv::imwrite( img_tmp, image_clone);
        }
    }

    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        cout << " camera info call back"<<endl;
        cinfo_ = info;
    }

    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
    {
        cout << " point cloud call back"<<endl;
        cloud_ = point_cloud;
	string tmp_pcd = pcd_folder+"/"+bag_filename+".pcd";
	cout << "pcdfilename" << tmp_pcd << endl;
	cloud_io::savePCDFile( tmp_pcd.c_str(), *cloud_);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExtractImgPcd");
    ExtractImgPcd node;

    if (system(NULL)) puts ("Ok");
    else exit (1);

    node.spin();
    return true;
}
