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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <cmath>
#include <vector>
#include <stdio.h>
#include <algorithm>

#define MAX_COST 255

using namespace std;

namespace door_base_collision_cost
{
  class DoorBaseCollisionCost
  {
    public:

    void transform2DInverse(const std::vector<double> &fp_in, std::vector<double> &fp_out, const double &door_x, const double &door_y, const double &door_theta);

    void transform2D(const std::vector<double> &fp_in, std::vector<double> &fp_out, const double &x, const double &y, const double &theta);

    double  findWorkspaceCost(const std::vector<double> robot_shoulder_position, const std::vector<double> robot_handle_position, const double &min_angle, const double &max_angle, const double &min_radius, const double &max_radius);

    bool  findAngleLimits(const double &door_length, const double &door_thickness, const double &pivot_length, const double max_radius, const std::vector<double> &point, std::vector<double> &angles);

    bool  findCircleLineSegmentIntersection(const std::vector<double> &p1, const std::vector<double> &p2, const double &x_r, const double &y_r, const double &radius, std::vector<std::vector<double> > intersection_points);

    void  freeAngleRange(const std::vector<std::vector<double> > &footprint, const double &door_length,  const double &door_thickness, const double &pivot_length, const double &max_radius, double &min_angle, double &max_angle);

    void  getValidDoorAngles(const std::vector<std::vector<double> > &footprint, 
                                                      const std::vector<double> &robot_global_pose, 
                                                      const std::vector<double> &door_global_pose, 
                                                      const std::vector<double> &robot_shoulder_position, 
                                                      const std::vector<double> &door_handle_pose, 
                                                      const double &door_length, const double &door_thickness, const double &pivot_length, 
                                                      const double &min_workspace_angle, const double &max_workspace_angle, 
                                                      const double &min_workspace_radius, const double &max_workspace_radius,
                                                      const double &delta_angle,
                                                      std::vector<int> &valid_angles, 
                             std::vector<int> &valid_cost);

    void  findCirclePolygonIntersection(const double &center_x, const double &center_y, const double &radius, const std::vector<std::vector<double> > &footprint, std::vector<std::vector<double> > &solution);

  };
}

