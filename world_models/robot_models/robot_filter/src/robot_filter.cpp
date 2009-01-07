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

/** \author Ioan Sucan */


#include <ros/node.h>
#include <robot_filter/RobotFilter.h>

namespace robot_filter {


  RobotFilter::RobotFilter(ros::node* node, std::string robot_model_name, bool verbose, double bodyPartScale) {
    m_model = new robot_model::NodeRobotModel(node, robot_model_name);
    m_node = node;
    m_verbose = verbose;
    m_bodyPartScale = bodyPartScale;
  }

  RobotFilter::~RobotFilter() {
    if (m_model) {
      delete m_model;
      m_model = NULL;
    }
    for (unsigned int i = 0 ; i < m_selfSeeParts.size() ; ++i)
      delete m_selfSeeParts[i].body;
  }
  
    
  void RobotFilter::loadRobotDescription() {
    m_model->loadRobotDescription();
  }

  void RobotFilter::waitForState() {
    m_model->waitForState();
  }
    
  void RobotFilter::addSelfSeeBodies(void)
  {
    robot_desc::URDF::Group *ss = m_model->getUrdf()->getGroup("self_see");
    if (ss && ss->hasFlag("collision"))
      {
	for (unsigned int i = 0 ; i < ss->linkNames.size() ; ++i)
	  {
	    planning_models::KinematicModel::Link *link = m_model->getKmodel()->getLink(ss->linkNames[i]);
	    if (link)
	      {
		RobotPart rp = { NULL, link };    
		    
		switch (link->shape->type)
		  {
		  case planning_models::KinematicModel::Shape::BOX:
		    rp.body = new collision_space::bodies::Box();
		    {
		      const double* size = static_cast<planning_models::KinematicModel::Box*>(link->shape)->size;
		      rp.body->setDimensions(size);
		    }
		    break;
		  case planning_models::KinematicModel::Shape::SPHERE:
		    rp.body = new collision_space::bodies::Sphere();
		    {
		      double size[1];
		      size[0] = static_cast<planning_models::KinematicModel::Sphere*>(link->shape)->radius;
		      rp.body->setDimensions(size);
		    }
		    break;
		  case planning_models::KinematicModel::Shape::CYLINDER:
		    rp.body = new collision_space::bodies::Cylinder();
		    {
		      double size[2];
		      size[0] = static_cast<planning_models::KinematicModel::Cylinder*>(link->shape)->length;
		      size[1] = static_cast<planning_models::KinematicModel::Cylinder*>(link->shape)->radius;
		      rp.body->setDimensions(size);
		    }
		    break;
		  default:
		    break;
		  }
		    
		if (!rp.body)
		  {
		    fprintf(stderr, "Unknown body type: %d\n", link->shape->type);
		    continue;
		  }
		    
		rp.body->setScale(m_bodyPartScale);
		    
		m_selfSeeParts.push_back(rp);
	      }
	  }
      }
	
    if (m_verbose)
      printf("Ignoring point cloud data that intersects with %d robot parts\n", m_selfSeeParts.size());
  }
  
  /** Remove points from the cloud if the robot sees parts of
      itself. Works for pointclouds in the robot frame. Creates
      a new cloud dynamically, so be sure to delete it. */
  std_msgs::PointCloud* RobotFilter::filter(const std_msgs::PointCloud &cloud)
  {

    if (m_selfSeeParts.empty() && m_model->getUrdf()) {
      addSelfSeeBodies();
    }
    
    if (!m_model->getRobotState()) {
      ROS_WARN("Ignoring state update because I haven't yet received the robot description");
    } else {
      m_model->stateUpdate();
      if (m_model->getKmodel())
	m_model->getKmodel()->computeTransforms(m_model->getRobotState()->getParams());
      if (m_model->getKmodelSimple())
	m_model->getKmodelSimple()->computeTransforms(m_model->getRobotStateSimple()->getParams());
    }

    if (cloud.header.frame_id != "map") {
      ROS_ERROR("Robot filter needs point clouds in the map frame. It was given a point cloud in the %s frame.",
		cloud.header.frame_id.c_str());
    }


    std_msgs::PointCloud *copy = new std_msgs::PointCloud();
    copy->header = cloud.header;
    

    
    for (int i = m_selfSeeParts.size() - 1 ; i >= 0 ; --i)
      m_selfSeeParts[i].body->setPose(m_selfSeeParts[i].link->globalTrans);
    

    unsigned int n = cloud.get_pts_size();
    unsigned int j = 0;
    copy->set_pts_size(n);	
    for (unsigned int k = 0 ; k < n ; ++k)
      {
	double x = cloud.pts[k].x;
	double y = cloud.pts[k].y;
	double z = cloud.pts[k].z;
	    
	
	bool keep = true;
	for (int i = m_selfSeeParts.size() - 1 ; keep && i >= 0 ; --i)
	  keep = !m_selfSeeParts[i].body->containsPoint(x, y, z);
	
	if (keep)
	  copy->pts[j++] = cloud.pts[k];
	
      }

    ROS_INFO("RobotFilter discarded %d points (%d left) \n", n - j, j);
	
    copy->set_pts_size(j);
    
    return copy;
  }
}



