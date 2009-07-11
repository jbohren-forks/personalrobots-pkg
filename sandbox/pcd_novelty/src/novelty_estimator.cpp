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

#include <novelty_estimator.h>

using namespace std;
using namespace robot_msgs;
using namespace pcd_novelty;


NoveltyEstimator::NoveltyEstimator()
{

  n_.param( std::string("~max_2d_search_radius"), max_2d_search_radius_, 0.1);
  n_.param( std::string("~max_3d_search_radius"), max_3d_search_radius_, 0.1);

  n_.param( std::string("~num_required_nei_2d"), num_required_nei_2d_, 3);
  n_.param( std::string("~num_required_nei_3d"), num_required_nei_3d_, 3);

  n_.param( std::string("~novelty_channel"), novelty_channel_,std::string("novelty"));
  ROS_INFO_STREAM("Novelty estimator: num 2d "<< num_required_nei_2d_ << " num 3d " << num_required_nei_3d_ );

}




void NoveltyEstimator::addCloudToHistory(const PointCloud &cloud)
{  
  boost::mutex::scoped_lock lock(proc_mutex_);

  unsigned int num_pts=cloud.pts.size();

  //Now lets make a 3d copy and 2d version of the cloud. 

  boost::shared_ptr<PcdHolder> pcd_holder=boost::shared_ptr<PcdHolder>(new PcdHolder());

  pcd_holder->cloud_2d_.pts.resize(num_pts);
  pcd_holder->cloud_3d_.pts.resize(num_pts);

  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    pcd_holder->cloud_3d_.pts[iPt]=cloud.pts[iPt];
    pcd_holder->cloud_2d_.pts[iPt]=cloud.pts[iPt];
    pcd_holder->cloud_2d_.pts[iPt].z=0;
  }

  //Now lets setup the search structures
  pcd_holder->kdtree_2d_=boost::shared_ptr<cloud_kdtree::KdTreeANN>(new cloud_kdtree::KdTreeANN(pcd_holder->cloud_2d_));
  pcd_holder->kdtree_3d_=boost::shared_ptr<cloud_kdtree::KdTreeANN>(new cloud_kdtree::KdTreeANN(pcd_holder->cloud_3d_));

  clouds_.push_back(pcd_holder);
}

void NoveltyEstimator::allocateNoveltyChannel(PointCloud& point_cloud,std::vector<float>** ptr_channel)
{
  int novelty_idx=cloud_geometry::getChannelIndex(point_cloud,novelty_channel_);
  if(novelty_idx==-1)
  {
    unsigned int num_channels = point_cloud.chan.size();
    point_cloud.chan.resize(num_channels+1);
    novelty_idx=num_channels;
    point_cloud.chan[novelty_idx].name=novelty_channel_;
    point_cloud.chan[novelty_idx].vals.resize(point_cloud.pts.size());
  }

  *ptr_channel = &point_cloud.chan[novelty_idx].vals;
}



void NoveltyEstimator::computeNovelty(const PointCloud& point_cloud,std::vector<float> &novelty_holder)
{
  ROS_INFO("Computing novelty");

  unsigned int inbound_cloud_size=point_cloud.pts.size();
  std::vector<float> relevant_cloud_count;
  std::vector<float> relevant_point_2d_count;
  std::vector<float> relevant_point_3d_count;
  relevant_cloud_count.resize(inbound_cloud_size);
  relevant_point_2d_count.resize(inbound_cloud_size);
  relevant_point_3d_count.resize(inbound_cloud_size);
  for(unsigned int iPt=0;iPt<inbound_cloud_size;iPt++)
  {
    novelty_holder[iPt]=0;
    relevant_cloud_count[iPt]=0;
    relevant_point_2d_count[iPt]=0;
    relevant_point_3d_count[iPt]=0;
  }


  for(PcdHolderDeque::iterator it=clouds_.begin();it!=clouds_.end();it++)
  {
    bool cloud_is_relevant=true;
    //Compute approximate relevance check
    if(!cloud_is_relevant)
    {
      continue;
    }

    unsigned int num_hits=0;
    for(unsigned int iPt=0;iPt<inbound_cloud_size;iPt++)
    {
      vector<int> k_indices;
      vector<float> k_distances;
      vector<int> k_indices3d;
      vector<float> k_distances3d;
      Point32 pt2d=point_cloud.pts[iPt];
      pt2d.z=0;
      (*it)->kdtree_2d_->radiusSearch (pt2d, max_2d_search_radius_, 
                                      k_indices, k_distances,num_required_nei_2d_);
      if( k_distances.size()<num_required_nei_2d_ )
      {
        continue;
      }
      num_hits++;
      relevant_cloud_count[iPt] ++;
      relevant_point_2d_count[iPt] ++ ;

      (*it)->kdtree_3d_->radiusSearch (point_cloud.pts[iPt], max_3d_search_radius_, 
                                      k_indices3d, k_distances3d,num_required_nei_3d_);
      if( k_distances3d.size()==num_required_nei_3d_ )
      {
        relevant_point_3d_count[iPt] ++ ;
      }

    }
    if( num_hits < inbound_cloud_size/2)
    {
      cloud_is_relevant=false;
      continue;
    }
  }

  for(unsigned int iPt=0;iPt<inbound_cloud_size;iPt++)
  {
    if( relevant_cloud_count[iPt]==0)
    {
      novelty_holder[iPt]=1;
      continue;
    }
    novelty_holder[iPt]  =  (relevant_point_2d_count[iPt] - relevant_point_3d_count[iPt])/ relevant_cloud_count[iPt];
  }
}

