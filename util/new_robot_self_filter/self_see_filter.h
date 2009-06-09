/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <filters/filter_base.h>
#include <ros/console.h>
#include <robot_msgs/PointCloud.h>
#include <planning_environment/robot_models.h>
#include <collision_space/bodies.h>
#include <tf/transform_listener.h>
#include <string>
#include <algorithm>

namespace filters
{

/** \brief A filter to remove parts of the robot seen in a pointcloud
 *
 */

template <typename T>
class SelfFilter: public FilterBase <T>
{

protected:

    struct SeeLink
    {
	std::string                    name;
	collision_space::bodies::Body* body;
	btTransform                    constTransf;
    };
     
    struct SortBodies
    {
	bool operator()(const SeeLink &b1, const SeeLink &b2)
	{
	    return b1.body->computeVolume() > b2.body->computeVolume();
	}
    };
    
public:
  /** \brief Construct the filter */
  SelfFilter(void) : rm_("robot_description"), tf_(*ros::Node::instance(), true, ros::Duration(10))
  {
      tf_.setExtrapolationLimit(ros::Duration().fromSec(10));      
  }
    

  /** \brief Destructor to clean up
   */
  ~SelfFilter(void)
  {
      for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	  if (bodies_[i].body)
	      delete bodies_[i].body;
  }
    
  virtual bool configure(void)
  {
      // keep only the points that are outside of the robot
      // for testing purposes this may be changed to true
      invert_ = false;
      
      std::vector<std::string> links = rm_.getSelfSeeLinks();
      double scale = rm_.getSelfSeeScale();
      double padd  = rm_.getSelfSeePadding();

      // from the geometric model, find the shape of each link of interest
      // and create a body from it, one that knows about poses and can 
      // check for point inclusion
      for (unsigned int i = 0 ; i < links.size() ; ++i)
      {
	  SeeLink sl;
	  sl.body = collision_space::bodies::createBodyFromShape(rm_.getKinematicModel()->getLink(links[i])->shape);
	  if (sl.body)
	  {
	      sl.name = links[i];
	      
	      // collision models may have an offset, in addition to what TF gives
	      // so we keep it around
	      sl.constTransf = rm_.getKinematicModel()->getLink(links[i])->constGeomTrans;
	      sl.body->setScale(scale);
	      sl.body->setPadding(padd);
	      bodies_.push_back(sl);
	  }
	  else
	      ROS_WARN("Unable to create point inclusion body for link '%s'", links[i].c_str());
      }
      
      if (bodies_.empty())
	  ROS_WARN("No robot links will be checked for self collision");
      
      // put larger volume bodies first -- higher chances of containing a point
      std::sort(bodies_.begin(), bodies_.end(), SortBodies());
      
      for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	  ROS_INFO("Self see link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].body->computeVolume());

      return true;
  }
    

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const robot_msgs::PointCloud& data_in, robot_msgs::PointCloud& data_out)
  {
      if (bodies_.empty())
	  data_out = data_in;
      else
      {
	  const unsigned int bs = bodies_.size();
	  const unsigned int np = data_in.pts.size();
	  std::vector<collision_space::bodies::BoundingSphere> bspheres(bs);
	  
	  // place the links in the frame of the pointcloud
	  for (unsigned int i = 0 ; i < bs ; ++i)
	  {
	      // find the transform between the link's frame and the pointcloud frame
	      tf::Stamped<btTransform> transf;
	      try
	      {
		  tf_.lookupTransform(data_in.header.frame_id, bodies_[i].name, data_in.header.stamp, transf);
	      }
	      catch(...)
	      {
		  transf.setIdentity();
		  ROS_ERROR("Unable to lookup transform from %s to %s", bodies_[i].name.c_str(), data_in.header.frame_id.c_str());
	      }
	      
	      // set it for each body; we also include the offset specified in URDF
	      bodies_[i].body->setPose(transf * bodies_[i].constTransf);
	      bodies_[i].body->computeBoundingSphere(bspheres[i]);
	  }
	  
	  // compute a sphere that bounds the entire robot
	  collision_space::bodies::BoundingSphere bound;
	  collision_space::bodies::mergeBoundingSpheres(bspheres, bound);	  

	  // we now decide which points we keep
	  std::vector<bool> keep(np);

#pragma omp parallel for 
	  for (int i = 0 ; i < (int)np ; ++i)
	  {
	      btVector3 pt = btVector3(btScalar(data_in.pts[i].x), btScalar(data_in.pts[i].y), btScalar(data_in.pts[i].z));
	      bool out = true;
	      if (bound.center.distance(pt) < bound.radius)
		  for (unsigned int j = 0 ; out && j < bs ; ++j)
		      out = !bodies_[j].body->containsPoint(pt);
	      
	      keep[i] = invert_ ? !out : out;
	  }
	  
	  
	  // fill in output data 
	  data_out.header = data_in.header;	  

	  data_out.pts.resize(0);
	  data_out.pts.reserve(np);
	  
	  data_out.chan.resize(data_in.chan.size());
	  for (unsigned int i = 0 ; i < data_out.chan.size() ; ++i)
	  {
	      ROS_ASSERT(data_in.chan[i].vals.size() == data_in.pts.size());
	      data_out.chan[i].name = data_in.chan[i].name;
	      data_out.chan[i].vals.reserve(data_in.chan[i].vals.size());
	  }
	  
	  for (unsigned int i = 0 ; i < np ; ++i)
	      if (keep[i])
	      {
		  data_out.pts.push_back(data_in.pts[i]);
		  for (unsigned int j = 0 ; j < data_out.chan.size() ; ++j)
		      data_out.chan[j].vals.push_back(data_in.chan[j].vals[i]);
	      }
      }
      return true;
  }
    
  virtual bool update(const std::vector<robot_msgs::PointCloud> & data_in, std::vector<robot_msgs::PointCloud>& data_out)
  {
      data_out.resize(data_in.size());
      for (unsigned int i = 0 ; i < data_in.size() ; ++i)
	  update(data_in[i], data_out[i]);
      return true;
  }
    
protected:
  
  planning_environment::RobotModels rm_;
  ros::NodeHandle                   nh_;  
  tf::TransformListener             tf_;
  std::vector<SeeLink>              bodies_;
  bool                              invert_;
  
};

typedef robot_msgs::PointCloud robot_msgs_PointCloud;
FILTERS_REGISTER_FILTER(SelfFilter, robot_msgs_PointCloud);

}

#endif //#ifndef FILTERS_SELF_SEE_H_
