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


#include <functional_m3n_ros/m3n_learning_node.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

M3NParams GLOBAL_m3n_params;

M3NParams& getM3NParameters(unsigned int nbr_clique_sets)
{
  vector<float> robust_potts_params(nbr_clique_sets, -1.0);

  // ----------------------------------------------
  // Define learning parameters
  RegressionTreeWrapperParams regression_tree_params;
  regression_tree_params.max_tree_depth_factor = 0.2; // was 0.4
  GLOBAL_m3n_params.setLearningRate(0.4);
  GLOBAL_m3n_params.setNumberOfIterations(6);
  GLOBAL_m3n_params.setRegressorRegressionTrees(regression_tree_params);
  GLOBAL_m3n_params.setInferenceRobustPotts(robust_potts_params);

  return GLOBAL_m3n_params;
}

using namespace std;
using namespace sensor_msgs;
using namespace m3n;


LearningNode::LearningNode()
{
  rf_creator_ = boost::shared_ptr<PtCloudRFCreator>(new  PtCloudRFCreator());

  cloud_sub_ = n_.subscribe<PointCloud>("training_cloud",100,boost::bind(&LearningNode::cloudCallback, this, _1));

  n_.param(std::string("~model_file_path"),model_file_path_,std::string("NONE"));
  if(model_file_path_=="NONE")
  {
    ROS_ERROR("Model file path is not specified");
    throw "ERR";
  }

  //model_pub_ = n_.advertise<ModelReference>("final_model_reference",100);

  learn_svc_ = n_.advertiseService(std::string("learn"), &LearningNode::learn,this);

  ROS_INFO("Learning node init done.");
}



void LearningNode::cloudCallback(const PointCloudConstPtr& the_cloud)
{
  ROS_INFO_STREAM("Got cloud "<< the_cloud->header.stamp << "@" << the_cloud->header.frame_id );
  training_clouds_.push_back(the_cloud);
}




int getAnnotationsChannel(const sensor_msgs::PointCloud& pt_cloud)
{
  int chan_annotation_idx=-1;
  for(unsigned int iC=0;iC<pt_cloud.channels.size();iC++)
  {

    const sensor_msgs::ChannelFloat32& chan = pt_cloud.channels[iC];

    //if (chan.name == "predictions" || ( (  chan.name.at(0)=='a'  ) && (  chan.name.at(1)=='n'  ) && chan.name.at(2)=='n'  ) ))
    if (( (  chan.name.at(0)=='a'  ) && (  chan.name.at(1)=='n'  ) && (  chan.name.at(2)=='n'  ) ))
    {
      chan_annotation_idx=iC;
      break;
    }
  }
  return chan_annotation_idx;
}

bool LearningNode::learn(functional_m3n_ros::Learn::Request  &req,
                         functional_m3n_ros::Learn::Response &res )
{
    ROS_INFO("Received learning command, starting to learn");

    boost::filesystem::path model_base_path(model_file_path_);
    if(~boost::filesystem::exists(model_base_path))
    {
      boost::filesystem::create_directory(model_base_path);
    }

    boost::filesystem::path model_path=model_base_path /= req.model_name;
    if(~boost::filesystem::exists(model_path))
    {
      boost::filesystem::create_directory(model_path);
    }

    boost::filesystem::path model_filename=model_path /= "f_m3n";

    std::string model_filename_str=model_filename.string();

    ROS_INFO_STREAM("Starting to learn model "<< model_filename_str);

    int num_clouds=training_clouds_.size();

    std::vector<boost::shared_ptr<RandomField> > rf_all;
    rf_all.resize(num_clouds);
    std::vector<const RandomField*> training_rfs;
    training_rfs.resize(num_clouds);

    int iCloud=0;
    for(std::deque<sensor_msgs::PointCloudConstPtr>::iterator cloud_it=training_clouds_.begin();cloud_it!=training_clouds_.end();cloud_it++,iCloud++)
    {

      ROS_INFO("Extracting cloud features");

      int chan_annotation_idx=getAnnotationsChannel(*(*cloud_it));
      if(chan_annotation_idx==-1)
      {
        ROS_ERROR("Annotations channel is missing");
        iCloud--;
        continue;
      }
      boost::shared_ptr<RandomField> rf=rf_creator_->createRandomField(*(*cloud_it),(*cloud_it)->channels[chan_annotation_idx].values);
      rf_all[iCloud]=rf; //mem management
      training_rfs[iCloud]=rf.get(); //for training
    }

    ROS_INFO_STREAM("Extracted features from "<< iCloud <<" clouds. Total "<< num_clouds <<" clouds.");
    rf_all.resize(iCloud);
    training_rfs.resize(iCloud);


    if(iCloud==0)
    {
      ROS_ERROR("No clouds to train");
      return false;
    }
    // ----------------------------------------------------------
    // Train M3N model
    ROS_INFO("Starting to train...");
    M3NModel m3n_model;

    learning_parameters_ = getM3NParameters(training_rfs[0]->getNumberOfCliqueSets());

    if (m3n_model.train(training_rfs, learning_parameters_) < 0)
    {
      ROS_ERROR("Failed to train M3N model");
      return false;
    }
    ROS_INFO("Successfully trained M3n model");
    m3n_model.saveToFile(model_filename_str);
    

    res.model_type="functional_m3n";
    res.model_reference=model_filename_str;

    return true;

}




/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "m3n_learning");

  LearningNode m3n_learning;

  ros::spin();

  return (0);
}
/* ]--- */


