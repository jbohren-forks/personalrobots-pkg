/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "ros/console.h"

#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"

class Cropper
{
  public:
    Cropper(double zmin, double zmax, const std::string& frame_id)
    {
      tf_ = new tf::TransformListener(*n.getNode());
      zmin_ = zmin;
      zmax_ = zmax;
      frame_id_ = frame_id;

      cloud_notifier_ =
              new tf::MessageNotifier<sensor_msgs::PointCloud>
              (tf_, n.getNode(), 
               boost::bind(&Cropper::cloudCallback, this, _1),
               "full_cloud", frame_id_, 100);

      pub_ = n.advertise<sensor_msgs::PointCloud>("full_cloud_cropped", 1);
    }

    ~Cropper()
    {
      delete cloud_notifier_;
      delete tf_;
    }
  private:
    ros::NodeHandle n;
    tf::TransformListener* tf_;
    tf::MessageNotifier<sensor_msgs::PointCloud>* cloud_notifier_;
    ros::Publisher pub_;

    double zmin_, zmax_;
    std::string frame_id_;

    void cloudCallback(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr& cloud_in)
    {
      ROS_INFO("Received message with %d points", cloud_in->pts.size());
      sensor_msgs::PointCloud cloud_transformed;
      tf_->transformPointCloud(frame_id_, *cloud_in, cloud_transformed);

      sensor_msgs::PointCloud cloud_out;
      cloud_out.header = cloud_transformed.header;
      for(unsigned int j=0;j<cloud_transformed.chan.size();j++)
      {
        sensor_msgs::ChannelFloat32 c;
        c.name = cloud_transformed.chan[j].name;
        cloud_out.chan.push_back(c);
      }
      for(unsigned int i=0;i<cloud_transformed.pts.size();i++)
      {
        if((cloud_transformed.pts[i].z >= zmin_) &&
           (cloud_transformed.pts[i].z <= zmax_))
        {
          cloud_out.pts.push_back(cloud_transformed.pts[i]);
          for(unsigned int j=0;j<cloud_transformed.chan.size();j++)
          {
            cloud_out.chan[j].vals.push_back(cloud_transformed.chan[j].vals[i]);
          }
        }
      }

      pub_.publish(cloud_out);
      ROS_INFO("Published message with %d points", cloud_out.pts.size());
    }
};

#define USAGE "USAGE point_cloud_cropper <zmin> <zmax> <frame>"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "cropper", ros::init_options::AnonymousName);
  if (argc != 4)
  {
    puts(USAGE);
    return 1;
  }

  Cropper c(atof(argv[1]), atof(argv[2]), argv[3]);
  ros::spin();
  return 0;
}
