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

#include <novelty_estimator_node.h>

using namespace std;
using namespace robot_msgs;
using namespace pcd_novelty;



NoveltyEstimatorNode::NoveltyEstimatorNode()
{
}



void NoveltyEstimatorNode::setup()
{
  boost::mutex::scoped_lock lock(proc_mutex_);  
  // Query map 

  cloud_pub_ = n_.advertise<sensor_msgs::PointCloud>("novelty_cloud",1);


  cloud_hist_sub_ = n_.subscribe<sensor_msgs::PointCloud>("cloud_hist",100,boost::bind(&NoveltyEstimatorNode::hist_cloudCallback, this,  _1));
  cloud_sub_ = n_.subscribe<sensor_msgs::PointCloud>("cloud",100,boost::bind(&NoveltyEstimatorNode::observed_cloudCallback, this,  _1));


  ROS_INFO_STREAM("Setup done.");
}

void NoveltyEstimatorNode::hist_cloudCallback(const sensor_msgs::PointCloudConstPtr& the_cloud)
{
  ROS_INFO("Adding cloud to history");
  estimator_.addCloudToHistory(*the_cloud);
}

void NoveltyEstimatorNode::observed_cloudCallback(const sensor_msgs::PointCloudConstPtr& the_cloud)
{
  sensor_msgs::PointCloud pcd_out=*the_cloud;
  std::vector<float>* ptr_channel;
  estimator_.allocateNoveltyChannel(pcd_out,&ptr_channel);
  estimator_.computeNovelty(pcd_out,*ptr_channel);

  int intensity_idx=cloud_geometry::getChannelIndex(pcd_out,"intensities");
  for(unsigned int iPt=0;iPt<pcd_out.pts.size();iPt++)
  {
    pcd_out.chan[intensity_idx].vals[iPt]=(*ptr_channel)[iPt];
  }

  cloud_pub_.publish(pcd_out);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv,"novelty_estimator_node");

  try
  {
    NoveltyEstimatorNode ne;
    ne.setup();

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}







