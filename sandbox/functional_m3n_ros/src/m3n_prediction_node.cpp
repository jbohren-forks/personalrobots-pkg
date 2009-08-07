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
using namespace sensor_msgs;
using namespace m3n;

#include "object_names/Float2Name.h"
#include "object_names/Name2Color.h"


PredictionNode::PredictionNode()
{

  rf_creator_ = boost::shared_ptr<PtCloudRFCreator>(new  PtCloudRFCreator());

  cloud_sub_ = n_.subscribe<PointCloud>("cloud",100,boost::bind(&PredictionNode::cloudCallback, this, _1));

  n_.param(std::string("~model_file"),model_file_name_,std::string("NONE"));

  n_.param( std::string("~use_color"), use_colors_, false);

  cloud_pub_ = n_.advertise<PointCloud>("predictions_cloud",100);

  n_.param(std::string("~ground_truth_channel"),ground_truth_channel_name_,std::string("NONE"));


  set_model_svc_ = n_.advertiseService(std::string("SetModel"), &PredictionNode::setModel,this);

  query_perf_stats_svc_  = n_.advertiseService(std::string("Performance"), &PredictionNode::queryPerformanceStats,this);

  nbr_correct=0;
  nbr_gt=0;
  total_accuracy=0.0;

  if(model_file_name_=="NONE")
  {
    has_model_=false;
    ROS_WARN("Model file is not specified. No predictions will be made.");
  }else if (m3n_model2.loadFromFile(model_file_name_) < 0)  
  {
    has_model_=false;
    ROS_ERROR("couldnt load model");
  }else{
    has_model_=true;

  }


  ROS_INFO("Init done.");

}
bool PredictionNode::queryPerformanceStats(
					   functional_m3n_ros::QueryPerformanceStats::Request  &req,
					   functional_m3n_ros::QueryPerformanceStats::Response &res )
{
  res.accuracy = total_accuracy;
  res.correct_weight = (double)nbr_correct;
  res.checked_weight = (double)nbr_gt;

  return true;
}

bool PredictionNode::setModel(functional_m3n_ros::SetModel::Request  &req,
			      functional_m3n_ros::SetModel::Response &res )
{
  model_file_name_=req.model_reference;
  ROS_INFO_STREAM("Loading new model "<<model_file_name_);
  if (m3n_model2.loadFromFile(model_file_name_) < 0)  
    {
      ROS_ERROR("couldnt load model");
      has_model_=false;
    }
  else
    {
      has_model_=true;
    }
  nbr_correct=0;
  nbr_gt=0;
  total_accuracy=0.0;

  return true;
}


void PredictionNode::cloudCallback(const PointCloudConstPtr& the_cloud)
{
  if(! has_model_)
    {
      ROS_ERROR("No model has been loaded. Call SetModel service or specify a valid model_file .");
      return;
    }
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
    
    int chan_gt=cloud_geometry::getChannelIndex(the_cloud,ground_truth_channel_name_);
    if(chan_gt!=-1)
      {
	unsigned int nbr_correct_in_pcd;
	unsigned int nbr_gt_in_pcd;
	double accuracy;	
	computeClassificationRates( cloud_out.chan[chan_predictions_id].vals, 
				    the_cloud->chan[chan_gt].vals,
				    m3n_model2.getTrainingLabels(),
				    nbr_correct_in_pcd,
				    nbr_gt_in_pcd,
				    accuracy);
	nbr_correct += nbr_correct_in_pcd;
	nbr_gt += nbr_gt_in_pcd;
	if(nbr_gt==0)
	  {
	    total_accuracy=0.0;
	  }
	else
	  {
	    total_accuracy = static_cast<double>(nbr_correct)/static_cast<double>(nbr_gt);
	  }

	ROS_INFO("Total correct: %u / %u = %f", nbr_correct, nbr_gt, total_accuracy);
    
      }
    

    cloud_pub_.publish(cloud_out);
}



void  PredictionNode::computeClassificationRates(const vector<float>& inferred_labels, const vector<float>& gt_labels, 
				    const vector<unsigned int>& labels,
				    unsigned int& nbr_correct,
				    unsigned int& nbr_gt,
				    double& accuracy)
    {
      // Initialize counters for each label
      // (map: label -> counter)
      std::map<unsigned int, unsigned int> total_label_count; // how many nodes with gt label
      std::map<unsigned int, unsigned int> correct_label_count; // how many correctly classified
      std::map<unsigned int, unsigned int> false_pos_label_count; // how many wrongly classified
      for (unsigned int i = 0 ; i < labels.size() ; i++)
      {
        total_label_count[labels[i]] = 0;
        correct_label_count[labels[i]] = 0;
        false_pos_label_count[labels[i]] = 0;
      }

      // Holds the total number of nodes correctly classified
      nbr_correct = 0;
      nbr_gt = 0;

      // Count the total and per-label number correctly classified
      unsigned int curr_node_id = 0;
      unsigned int curr_gt_label = 0;
      unsigned int curr_infer_label = 0;
      vector<unsigned int>::const_iterator iter_predictions;
      for (unsigned int i=0;i<inferred_labels.size();i++)
      {
	curr_gt_label = (unsigned int)gt_labels[i];
        curr_infer_label = (unsigned int)inferred_labels[i];

        total_label_count[curr_gt_label]++;
	if(curr_gt_label==0)
	  {
	    continue;
	  }

	nbr_gt++;
        if (curr_gt_label == curr_infer_label)
        {
          nbr_correct++;
          correct_label_count[curr_gt_label]++;
        }
        else
        {
          false_pos_label_count[curr_infer_label]++;
        }
      }

      // Print statistics
      if(nbr_gt==0)
	{
	  accuracy=0.0;
	}
      else
	{
	  accuracy = static_cast<double>(nbr_correct)/static_cast<double>(nbr_gt);
	}
    
      
      ROS_INFO("Total correct: %u / %u = %f", nbr_correct, nbr_gt, accuracy);
      stringstream ss;
      ss << "Label distribution: ";
      unsigned int curr_label = 0;
      for (unsigned int i = 0 ; i < labels.size() ; i++)
      {
        curr_label = labels[i];
        ss << "[" << curr_label << ": " << correct_label_count[curr_label] << "/"
            << total_label_count[curr_label] << " (" << false_pos_label_count[curr_label] << ")]  ";
      }
      ROS_INFO("%s", ss.str().c_str());
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

