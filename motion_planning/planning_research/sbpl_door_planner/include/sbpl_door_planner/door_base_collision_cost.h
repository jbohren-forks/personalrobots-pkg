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

#include <robot_msgs/Point32.h>

#define MAX_COST 255

using namespace std;

namespace door_base_collision_cost
{
  class DoorBaseCollisionCost
  {

    public:


    void init();

    void getValidDoorAngles(const robot_msgs::Point32 &global_position, const double &global_yaw, std::vector<int> &valid_angle, std::vector<int> &valid_cost);


    std::vector<robot_msgs::Point32> footprint_;/*! 2D footprint of the robot */

    double arm_min_workspace_radius_; /*! min allowable radius of the workspace from the shoulder position of the arm */

    double arm_max_workspace_radius_;  /*! max allowable radius of the workspace from the shoulder position of the arm */

    double arm_min_workspace_angle_;  /*! min allowable workspace angle from the shoulder position of the arm */

    double arm_max_workspace_angle_;  /*! max allowable workspace angle from the shoulder position of the arm */

    robot_msgs::Point32 door_frame_global_position_; /*! Global 2D position of the door frame */

    double door_frame_global_yaw_;

    robot_msgs::Point32 robot_shoulder_position_; /*! 2D position of the shoulder of the arm used for opening/closing the door in the robot frame ("base_link") frame */

    robot_msgs::Point32 door_handle_position_; /*! 2D position of the handle in a frame attached to the door at the hinge with X axis towards the free end of the door and Z pointing upwards */

    double door_thickness_; /*! Thickness of the door (in meters) */

    double pivot_length_; /*! Length from pivot/hinge to closest edge of the door */

    double door_length_; /*! Length of the door (in meters) */

    double door_angle_discretization_interval_; /*! Discretization interval used by the planner to open and close doors */

    double global_door_open_angle_;  /*! Angle of the door when open (in global frame) */

    double global_door_closed_angle_; /*! Angle of the door when closed (in global frame) */

    int rot_dir_; /*! Rotation direction to OPEN the door (w.r.t Z axis pointing upwards) */

    double max_door_collision_radius_;

    void writeToFile(std::string filename);

    void writeSolution(const std::string &filename, const robot_msgs::Point32 &robot_position, const double &robot_yaw, const std::vector<int> &angles, const std::vector<int> &angle_costs);

    private:

    void transform2DInverse(const robot_msgs::Point32 &point_in, 
                                                   robot_msgs::Point32 &point_out, 
                                                   const robot_msgs::Point32 &frame, 
                                                   const double &frame_yaw);

    void transform2D(const robot_msgs::Point32 &point_in, robot_msgs::Point32 &point_out, const robot_msgs::Point32 &frame, const double &frame_yaw);


    unsigned char findWorkspaceCost(const robot_msgs::Point32 &robot_position, 
                                    const double &robot_yaw, 
                                    const double &door_angle);

    bool  findAngleLimits(const double max_radius, const robot_msgs::Point32 &point, double &angle);


    bool findCircleLineSegmentIntersection(const robot_msgs::Point32 &p1, 
                                           const robot_msgs::Point32 &p2, 
                                           const robot_msgs::Point32 &center, 
                                           const double &radius, 
                                           std::vector<robot_msgs::Point32> &intersection_points);

    void findCirclePolygonIntersection(const robot_msgs::Point32 &center, 
                                       const double &radius, 
                                       const std::vector<robot_msgs::Point32> &footprint, 
                                       std::vector<robot_msgs::Point32> &solution);

    void freeAngleRange(const std::vector<robot_msgs::Point32> &footprint, 
                        const double &max_radius, 
                        double &min_obstructed_angle, 
                        double &max_obstructed_angle);


    void getDoorFrameFootprint(const robot_msgs::Point32 &robot_global_position, const double &robot_global_yaw, std::vector<robot_msgs::Point32> &fp_out);


    double local_door_open_angle_;  /*! Angle of the door when open (in local door frame) */

    double local_door_closed_angle_; /*! Angle of the door when closed (in local door frame) */

    double local_door_min_angle_;

    double local_door_max_angle_;

  };
}

