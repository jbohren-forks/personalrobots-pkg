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


#include <functional_m3n_ros/m3n_prediction_node.h>

using namespace std;
using namespace robot_msgs;
using namespace m3n;

#include "object_names/Float2Name.h"
#include "object_names/Name2Color.h"


PredictionNode::PredictionNode()
{

  rf_creator_ = boost::shared_ptr<PtCloudRFCreator>(new  PtCloudRFCreator());

  cloud_sub_ = n_.subscribe<PointCloud>("cloud",100,boost::bind(&PredictionNode::cloudCallback, this, _1));

  n_.param(std::string("~model_file"),model_file_name_,std::string("NONE"));
  if(model_file_name_=="NONE")
  {
    ROS_ERROR("Model file is not specified");
    throw "ERR";
  }

  n_.param( std::string("~use_color"), use_colors_, false);

  cloud_pub_ = n_.advertise<PointCloud>("predictions_cloud",100);


  if (m3n_model2.loadFromFile(model_file_name_) < 0)  
  {
    ROS_ERROR("couldnt load model");
    throw "ERR";
  }
  ROS_INFO("Init done.");
}



void PredictionNode::cloudCallback(const PointCloudConstPtr& the_cloud)
{

    ROS_INFO("Received point cloud, starting detection");
    //cloud_ = the_cloud;

    boost::shared_ptr<RandomField> rf=rf_creator_->createRandomField(*the_cloud);


    map<unsigned int, unsigned int> inferred_labels;
    if (m3n_model2.infer(*rf, inferred_labels) < 0)
    {
      ROS_ERROR("could not do inference");
      return;
    }


    PointCloud cloud_out=*the_cloud;

    unsigned int num_in=the_cloud->pts.size();
    unsigned int nC_in=the_cloud->chan.size();
    unsigned int free_channel=the_cloud->chan.size();

    int chan_RGB_id;
    map<int, std_msgs::ColorRGBA> label_colors;
    if(use_colors_)
    {
      chan_RGB_id=cloud_geometry::getChannelIndex(cloud_out,"rgb");
      if(chan_RGB_id==-1)
      {
        chan_RGB_id = free_channel;
       free_channel++;
      }
      const vector<unsigned int> labels=m3n_model2.getTrainingLabels();
      for(int i=0;i<labels.size();i++)
      {
        object_names::Float2Name::Request req1;
        object_names::Float2Name::Response resp1;
        req1.id=labels[i];
        
        ros::service::call("float_to_name",req1,resp1);

        object_names::Name2Color::Request req2;
        object_names::Name2Color::Response resp2;
        req2.name=resp1.name;
        
        ros::service::call("name_to_color",req2,resp2);

        label_colors[labels[i]]=resp2.color;
      }
    }

    unsigned int chan_predictions_id=free_channel;
    free_channel++;
    unsigned int nCout=free_channel;
    cloud_out.chan.resize(nCout);

    cloud_out.chan[chan_predictions_id].name="predicted_label";
    cloud_out.chan[chan_predictions_id].vals.resize(num_in);

    if(use_colors_ && chan_RGB_id>=nC_in)
    {
      cloud_out.chan[chan_RGB_id].name="rgb";
      cloud_out.chan[chan_RGB_id].vals.resize(num_in);
    }


    for(unsigned int iPt=0;iPt<num_in;iPt++)
    {
      cloud_out.chan[chan_predictions_id].vals[iPt]=inferred_labels[iPt];

      if(use_colors_)
      {
        const std_msgs::ColorRGBA& color=label_colors[inferred_labels[iPt]];
        int r=int(round(color.r*255));
        int g=int(round(color.g*255));
        int b=int(round(color.b*255));
        int rgb = (r << 16) | (g << 8) | b;
        cloud_out.chan[chan_RGB_id].vals[iPt] = *reinterpret_cast<float*>(&rgb);

      }
    }        

    cloud_pub_.publish(cloud_out);
}




/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "m3n_prediction");


  PredictionNode m3n_prediction;

  ros::spin();

  return (0);
}
/* ]--- */

