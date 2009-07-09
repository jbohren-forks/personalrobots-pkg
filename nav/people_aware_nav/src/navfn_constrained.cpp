/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <people_aware_nav/navfn_constrained.h>
#include <ros/ros.h>

namespace people_aware_nav {

using robot_msgs::PoseStamped;
using std::vector;
using robot_msgs::Point;
using robot_msgs::Point32;

ROS_REGISTER_BGP(NavfnROSConstrained);

NavfnROSConstrained::NavfnROSConstrained (std::string name, costmap_2d::Costmap2DROS& cmap) :
  navfn::NavfnROS(name, cmap)
{
  service_ = node_.advertiseService("~set_nav_constraint", &NavfnROSConstrained::setConstraint, this);
}

bool NavfnROSConstrained::setConstraint (SetNavConstraint::Request& req, SetNavConstraint::Response& resp)
{
  forbidden_ = req.forbidden;
  ROS_DEBUG_STREAM_NAMED("navfn", "Setting constraint polygon with " << forbidden_.points.size() << " points");
  return true;
}


void NavfnROSConstrained::getCostmap (costmap_2d::Costmap2D& cmap)
{
  costmap_ros_.clearRobotFootprint();
  costmap_ros_.getCostmapCopy(cmap);

  // Set cost of forbidden region
  vector<Point> polygon;
  for (vector<Point32>::const_iterator iter = forbidden_.points.begin(); iter!=forbidden_.points.end(); ++iter) {
    Point p;
    p.x = iter->x;
    p.y = iter->y;
    p.z = iter->z;
    ROS_DEBUG_STREAM("Adding constraint point " << p.x << ", " << p.y << ", " << p.z);
    polygon.push_back(p);
  }
  cmap.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);

  ROS_INFO ("Modified costmap to take constraints into account");
}






} // namespace
