/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#ifndef SPLINE_SMOOTHER_H_
#define SPLINE_SMOOTHER_H_

#include <vector>
#include <manipulation_msgs/WaypointTraj.h>
#include <filters/filter_base.h>

namespace spline_smoother
{

/**
 * \brief Abstract base class for spline smoothing.
 *
 * To implement a smoother, just override the virtual "smooth" method, and call the
 * REGISTER_SPLINE_SMOOTHER macro with the class name (anywhere in the cpp file)
 */
class SplineSmoother: public filters::FilterBase<manipulation_msgs::WaypointTraj>
{
public:
  SplineSmoother();
  virtual ~SplineSmoother();

  virtual bool configure();

  virtual bool update(const std::vector<manipulation_msgs::WaypointTraj>& data_in, std::vector<manipulation_msgs::WaypointTraj>& data_out);

  /**
   * \brief Smooths the input position trajectory by generating velocities and accelerations at the waypoints.
   *
   * \return true if successful, false if not
   */
  virtual bool smooth(const manipulation_msgs::WaypointTraj& trajectory_in, manipulation_msgs::WaypointTraj& trajectory_out) const = 0;
};

}

typedef manipulation_msgs::WaypointTraj manipulation_msgs__WaypointTraj;

#define REGISTER_SPLINE_SMOOTHER(c) \
  FILTERS_REGISTER_FILTER_NONTEMPLATE(c, manipulation_msgs__WaypointTraj)

#endif /* SPLINE_SMOOTHER_H_ */
