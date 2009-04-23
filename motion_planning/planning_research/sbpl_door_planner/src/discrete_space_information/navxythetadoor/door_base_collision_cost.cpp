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

#include <sbpl_door_planner/door_base_collision_cost.h>
#include <robot_msgs/Point32.h>
#include <angles/angles.h>


#define MAX_COST 255
//#define DEBUG 1
using namespace std;

namespace door_base_collision_cost
{

  void DoorBaseCollisionCost::transform2DInverse(const robot_msgs::Point32 &point_in, 
                                                 robot_msgs::Point32 &point_out, 
                                                 const robot_msgs::Point32 &frame, 
                                                 const double &frame_yaw)
  {
    double cth = cos(frame_yaw);
    double sth = sin(frame_yaw);

    point_out.x = point_in.x * cth + point_in.y * sth - frame.x * cth - frame.y * sth;
    point_out.y = -point_in.x * sth + point_in.y * cth + frame.x * sth - frame.y * cth;
    return;
  }

  void DoorBaseCollisionCost::transform2D(const robot_msgs::Point32 &point_in, robot_msgs::Point32 &point_out, const robot_msgs::Point32 &frame, const double &frame_yaw)
  {
    double cth = cos(frame_yaw);
    double sth = sin(frame_yaw);

    point_out.x = frame.x + point_in.x * cth - point_in.y * sth;
    point_out.y = frame.y + point_in.x * sth + point_in.y * cth;
    return;
  }



  unsigned char DoorBaseCollisionCost::findWorkspaceCost(const robot_msgs::Point32 &robot_position, 
                                                         const double &robot_yaw, 
                                                         const double &door_angle)
  {

    robot_msgs::Point32 global_handle_position;
    robot_msgs::Point32 robot_handle_position;

    //Transform the handle position
    transform2D(door_handle_position_,global_handle_position,door_frame_global_position_,door_frame_global_yaw_+door_angle);
    transform2DInverse(global_handle_position,robot_handle_position,robot_position, robot_yaw);

    double dx = robot_handle_position.x - robot_shoulder_position_.x;
    double dy = robot_handle_position.y - robot_shoulder_position_.y;

    double d = sqrt(dx*dx + dy*dy);

#ifdef DEBUG
    printf("\nWorkspace computation\n");
    printf("Robot position: %f %f %f\n",robot_position.x,robot_position.y,robot_yaw);
    printf("Door angle: %f\n",door_angle);
    printf("Global handle position: %f, %f\n",global_handle_position.x,global_handle_position.y);
    printf("Robot handle position: %f, %f\n\n",robot_handle_position.x,robot_handle_position.y);
#endif

    if(d > arm_max_workspace_radius_ || d < arm_min_workspace_radius_)
    {
#ifdef DEBUG
      printf("handle too far from shoulder\n\n");
#endif
      return MAX_COST;
    }

    double angle = atan2(dy,dx);
    if(angle > arm_max_workspace_angle_ || angle < arm_min_workspace_angle_)
    {
#ifdef DEBUG
      printf("handle: %f, (dx,dy) (%f,%f) too acute from shoulder\n\n",angle,dx,dy);
#endif
      return MAX_COST;
    }
    else
    {
      double factor = 1.0/(1.0 + pow((angle - (arm_max_workspace_angle_+arm_min_workspace_angle_)/2.0),2) * pow((d - (arm_max_workspace_radius_ + arm_min_workspace_radius_)/2.0),2));
      return (unsigned char)((1-factor)*MAX_COST);
    }
  }

  bool  DoorBaseCollisionCost::findAngleLimits(const double max_radius, const robot_msgs::Point32 &point, double &angle)
  {
    double eps_distance = 0.001;
    double dx = point.x;
    double dy = point.y;
    double d = sqrt(dx*dx + dy*dy);
    if(d > (max_radius+eps_distance))
    {
#ifdef DEBUG
      printf("Distance too high %f, max: %f\n",d,max_radius);
#endif
      return false;
    }
    angle = atan2(dy,dx);
    return true;
  }


  bool DoorBaseCollisionCost::findCircleLineSegmentIntersection(const robot_msgs::Point32 &p1, 
                                                                const robot_msgs::Point32 &p2, 
                                                                const robot_msgs::Point32 &center, 
                                                                const double &radius, 
                                                                std::vector<robot_msgs::Point32> &intersection_points)
  {
    double eps = 1e-5;
    double dx = p2.x-p1.x;
    double dy = p2.y-p1.y;
    double theta = atan2(dy,dx);
    double cost = cos(theta);
    double sint = sin(theta);
    double d = sqrt(dx*dx+dy*dy);

    if(d < eps)
    {
#ifdef DEBUG
      printf("No intersection found\n");
#endif
      return false;
    }

    robot_msgs::Point32 int_pt;

    double a = pow(d,2);
    double b = -2*center.x*d*cost + 2*p1.x*d*cost -2*center.y*d*sint + 2*p1.y*d*sint;
    double c = pow(p1.x,2) + pow(center.x,2) - 2*p1.x*center.x + pow(p1.y,2) + pow(center.y,2) - 2*p1.y*center.y - pow(radius,2);

    double disc = pow(b,2) - 4*a*c;
    if(disc < 0)
      return false;

    double t1 = (-b + sqrt(disc))/(2*a);   
    if(t1 >= 0 && t1 <= 1) // point is inside limits on the line segment
    {
      int_pt.x = p1.x+d*cost*t1;
      int_pt.y = p1.y+d*sint*t1;
      intersection_points.push_back(int_pt);
    }

    t1 = (-b - sqrt(disc))/(2*a);   
    if(t1 >= 0 && t1 <= 1) // point is inside limits on the line segment
    {
      int_pt.x = p1.x+d*cost*t1;
      int_pt.y = p1.y+d*sint*t1;
      intersection_points.push_back(int_pt);
    }
    return true;
  }


  void DoorBaseCollisionCost::findCirclePolygonIntersection(const robot_msgs::Point32 &center, 
                                                            const double &radius, 
                                                            const std::vector<robot_msgs::Point32> &footprint, 
                                                            std::vector<robot_msgs::Point32> &solution)
  {
    std::vector<robot_msgs::Point32> intersection_points;

    for(int i=0; i < (int) footprint.size(); i++)
    {
      int p1 = i;
      int p2 = (i+1)%((int) footprint.size());      
      intersection_points.resize(0);
#ifdef DEBUG
      printf("\nPoints: %d %d, %f %f, %f %f, \ncenter: %f, %f\n radius: %f\n",p1,p2,footprint[p1].x,footprint[p1].y,footprint[p2].x,footprint[p2].y,center.x,center.y,radius);
#endif
      if (findCircleLineSegmentIntersection(footprint[p1],footprint[p2],center,radius,intersection_points))
      {
        for(int j=0; j< (int) intersection_points.size(); j++)
        {
#ifdef DEBUG
          printf("soln:%d:: %f, %f\n",j,intersection_points[j].x,intersection_points[j].y);
          printf("angle: %f\n",atan2(intersection_points[j].y,intersection_points[j].x)*180.0/M_PI);
#endif
          solution.push_back(intersection_points[j]);
        }
      }
    }
  }

  void DoorBaseCollisionCost::freeAngleRange(const std::vector<robot_msgs::Point32> &footprint, 
                                             const double &max_radius, 
                                             double &min_obstructed_angle, 
                                             double &max_obstructed_angle)
  {
    std::vector<double> obstructed_angles;
    double angle(0.0);

    for(int i=0; i < (int) footprint.size(); i++)
    {
      if(findAngleLimits(max_radius,footprint[i],angle))
      {
#ifdef DEBUG
        printf("Angle vertex %d: %f\n",i,angle*180.0/M_PI); 
        if(isnan(angle))
        {
          printf("fp\n");
          exit(-1);
        }
#endif
          obstructed_angles.push_back(angle);
      }
    }
    std::vector<robot_msgs::Point32> solution;
    robot_msgs::Point32 center;
    center.x = 0.0;
    center.y = 0.0;
    findCirclePolygonIntersection(center,max_radius,footprint, solution);
    for(int i=0; i < (int) solution.size(); i++)
    {
      if(findAngleLimits(max_radius,solution[i],angle))
      {
#ifdef DEBUG
        printf("Angle edge %d: %f\n",i,angle*180.0/M_PI); 
        if(isnan(angle))
        {
          printf("circle: %f %f\n radius: %f\n %f %f\n",center.x,center.y,max_radius,footprint[i].x,footprint[i].y);
          exit(-1);
        }
#endif
        obstructed_angles.push_back(angle);
      }
    }

    if(obstructed_angles.size() < 1) //The door swept area is collision free
    {
      min_obstructed_angle = local_door_max_angle_;
      max_obstructed_angle = local_door_min_angle_;
      return;
    }

#ifdef DEBUG
    printf("\n\n\n");
    for(int i=0; i<(int)obstructed_angles.size(); i++)
    {
      printf("obs angle: %d, %f\n",i,obstructed_angles[i]*180.0/M_PI);
    }
#endif


    std::sort(obstructed_angles.begin(),obstructed_angles.end());
    min_obstructed_angle = obstructed_angles[0];
    max_obstructed_angle = obstructed_angles[obstructed_angles.size()-1];

    if(isnan(max_obstructed_angle))
    {
      printf("Size: %d\n",obstructed_angles.size());
      exit(-1);
    }
    return;
  }


  void DoorBaseCollisionCost::getDoorFrameFootprint(const robot_msgs::Point32 &robot_global_position, const double &robot_global_yaw, std::vector<robot_msgs::Point32> &fp_out)
  {
    std::vector<robot_msgs::Point32> global_fp;
    global_fp.resize(footprint_.size());
    for(int i=0; i < (int) footprint_.size(); i++)
    {
      transform2D(footprint_[i],global_fp[i],robot_global_position,robot_global_yaw);
      transform2DInverse(global_fp[i],fp_out[i],door_frame_global_position_,door_frame_global_yaw_);
    }
  }

  void DoorBaseCollisionCost::init()
  {
    max_door_collision_radius_ = sqrt(pow(door_length_,2) + pow(door_thickness_ + pivot_length_,2));
//local open/close angles should now be in the -pi/2 to pi/2 range (subject to small sensor error) 
    local_door_open_angle_   = angles::shortest_angular_distance(door_frame_global_yaw_,global_door_open_angle_);
    local_door_closed_angle_ = angles::shortest_angular_distance(door_frame_global_yaw_,global_door_closed_angle_);
    local_door_min_angle_ = std::min<double>(local_door_open_angle_,local_door_closed_angle_);
    local_door_max_angle_ = std::max<double>(local_door_open_angle_,local_door_closed_angle_);      
  }

  void DoorBaseCollisionCost::getValidDoorAngles(const robot_msgs::Point32 &global_position, const double &global_yaw, std::vector<int> &valid_angles, std::vector<int> &valid_cost, std::vector<unsigned char> &valid_interval) 
  {
    double min_obstructed_angle(0.0),max_obstructed_angle(0.0);
    std::vector<robot_msgs::Point32> door_fp;

    door_fp.resize(footprint_.size());
    getDoorFrameFootprint(global_position,global_yaw,door_fp);
    freeAngleRange(door_fp,max_door_collision_radius_,min_obstructed_angle,max_obstructed_angle);
#ifdef DEBUG
    printf("\nMin %f; Max %f\n",min_obstructed_angle,max_obstructed_angle);
    printf("\nLocal min: %f max: %f\n",local_door_min_angle_,local_door_max_angle_);
#endif
    if(min_obstructed_angle > local_door_min_angle_)
    {
      if(min_obstructed_angle >= local_door_max_angle_)
        min_obstructed_angle = local_door_max_angle_;

      int num_intervals = (int) (angles::normalize_angle(min_obstructed_angle-local_door_min_angle_)/door_angle_discretization_interval_);
      for(int i=0; i<num_intervals; i++)
      {
        double new_angle = angles::normalize_angle(local_door_min_angle_ + i * door_angle_discretization_interval_);
        unsigned char cost = findWorkspaceCost(global_position,global_yaw,new_angle);
        if(cost < MAX_COST)
        {
          if(min_obstructed_angle == local_door_max_angle_)
          {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(1);
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(0);
          }
          else
          {
            if(local_door_min_angle_ == local_door_open_angle_)
            {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(1);
            }
            else if(local_door_min_angle_ == local_door_closed_angle_)
            {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(0);
            }
          }
#ifdef DEBUG
          printf("Interval: %d\n",valid_interval.back());
          printf("Min interval, rot_dir:%d num_intervals:%d\n",rot_dir_,num_intervals);
          printf("\nMin %f; Max %f\n",min_obstructed_angle,max_obstructed_angle);
          printf("\nLocal min: %f max: %f\n",local_door_min_angle_,local_door_max_angle_);
#endif
        }
      }
    }

    if(max_obstructed_angle < local_door_max_angle_)
    {
      if(max_obstructed_angle <= local_door_min_angle_)
      {
        max_obstructed_angle = local_door_min_angle_;
      }
      int num_intervals = (int) (angles::normalize_angle(local_door_max_angle_-max_obstructed_angle)/door_angle_discretization_interval_);
      for(int i=0; i<num_intervals; i++)
      {
        double new_angle = angles::normalize_angle(local_door_max_angle_ - i * door_angle_discretization_interval_);
        unsigned char cost = findWorkspaceCost(global_position,global_yaw,new_angle);
        if(cost < MAX_COST)
        {
          if(max_obstructed_angle == local_door_min_angle_)
          {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(1);
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(0);
          }
          else
          {
            if(local_door_max_angle_ == local_door_open_angle_)
            {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(1);
            }
            else if(local_door_max_angle_ == local_door_closed_angle_)
            {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int) cost);
              valid_interval.push_back(0);
            }
          }
#ifdef DEBUG
          printf("Interval: %d\n",valid_interval.back());
          printf("Max interval, rot_dir:%d\n",rot_dir_);
          printf("\nMin %f; Max %f\n",min_obstructed_angle,max_obstructed_angle);
          printf("\nLocal min: %f max: %f\n",local_door_min_angle_,local_door_max_angle_);
#endif
        }
      }
    }
  }

  void DoorBaseCollisionCost::writeToFile(std::string filename)
  {
    FILE *fp = fopen(filename.c_str(),"wt");

    //Print door position
    fprintf(fp,"hinge_global_position = [%f %f];\n",door_frame_global_position_.x,door_frame_global_position_.y);    
    fprintf(fp,"door_length = %f\n",door_length_);
    fprintf(fp,"door_angle = %f\n",door_frame_global_yaw_);
    fprintf(fp,"local_footprint = [ ");

    for(int i=0; i < (int) footprint_.size(); i++)
    {
      fprintf(fp,"%f %f\n",footprint_[i].x,footprint_[i].y);
    }

    fprintf(fp,"];");
    fclose(fp);
  }

  void DoorBaseCollisionCost::writeSolution(const std::string &filename, const robot_msgs::Point32 &robot_position, const double &robot_yaw, const std::vector<int> &angles, const std::vector<int> &angle_costs)
  {
    FILE *fp = fopen(filename.c_str(),"wt");
    //Print door position
    for(int i=0; i < (int) angles.size(); i++)
    {
      fprintf(fp,"%f %f %f %f %f\n",robot_position.x, robot_position.y, robot_yaw, angles::normalize_angle((double)angles[i]*M_PI/180.0 + door_frame_global_yaw_),(double) angle_costs[i]);
    }
    fclose(fp);
  }

  void DoorBaseCollisionCost::getDesiredDoorAngles(const std::vector<int> &desired_door_anglesV, std::vector<int> &local_desired_door_angles)
  {
    //Convert global closed angle to degrees
    for(int i=0; i < (int) desired_door_anglesV.size(); i++)
    {
      local_desired_door_angles[i] = (int) angles::to_degrees(angles::shortest_angular_distance(global_door_closed_angle_,angles::from_degrees(desired_door_anglesV[i])));
//      printf("\n\nDB:: desired door angle: %d\n\n",desired_door_anglesV[i]);
//      printf("\n\nDB:: desired door angle: %d\n\n",local_desired_door_angles[i]);
    }
  }
};


