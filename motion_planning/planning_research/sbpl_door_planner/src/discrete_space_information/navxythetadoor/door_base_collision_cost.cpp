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
#define MAX_COST 255
//#define DEBUG 1
using namespace std;

namespace door_base_collision_cost
{
      void DoorBaseCollisionCost::transform2DInverse(const std::vector<double> &fp_in, std::vector<double> &fp_out, const double &door_x, const double &door_y, const double &door_theta)
      {
        double cth = cos(door_theta);
        double sth = sin(door_theta);

//        fp_out.resize(2);
        fp_out.push_back(fp_in[0]*cth+fp_in[1]*sth-door_x*cth-door_y*sth);
        fp_out.push_back(-fp_in[0]*sth+fp_in[1]*cth+door_x*sth-door_y*cth);
        return;
      }
      void  DoorBaseCollisionCost::transform2D(const std::vector<double> &fp_in, std::vector<double> &fp_out, const double &x, const double &y, const double &theta)
      {
        double cth = cos(theta);
        double sth = sin(theta);

//        fp_out.resize(2);
        fp_out.push_back(fp_in[0]*cth-fp_in[1]*sth+x);
        fp_out.push_back(fp_in[0]*sth+fp_in[1]*cth+y);
        return;
      }

      double  DoorBaseCollisionCost::findWorkspaceCost(const std::vector<double> robot_shoulder_position, const std::vector<double> robot_handle_position, const double &min_angle, const double &max_angle, const double &min_radius, const double &max_radius)
      {
        double dx = robot_handle_position[0] - robot_shoulder_position[0];
        double dy = robot_handle_position[1] - robot_shoulder_position[1];

        double d = sqrt(dx*dx + dy*dy);

        if(d > max_radius || d < min_radius)
        {
#ifdef DEBUG
          printf("handle too far from shoulder\n\n");
#endif
          return MAX_COST;
        }

        double angle = atan2(dy,dx);

        if(angle > max_angle || angle < min_angle)
        {
#ifdef DEBUG
          printf("handle: %f, (dx,dy) (%f,%f) too acute from shoulder\n\n",angle,dx,dy);
#endif
          return MAX_COST;
        }
        else
        {
          double factor = 1.0/(1.0 + pow((angle - (max_angle+min_angle)/2.0),2) * pow((d - (max_radius+min_radius)/2.0),2));
          return (unsigned char)((1-factor)*MAX_COST);
        }
      }

      bool  DoorBaseCollisionCost::findAngleLimits(const double &door_length, const double &door_thickness, const double &pivot_length, const double max_radius, const std::vector<double> &point, std::vector<double> &angles)
      {
        double dx = point[0];
        double dy = point[1];
        double d = sqrt(dx*dx + dy*dy);

        if(d > max_radius)
        {
          return false;
        }

        double t3 = atan2(dy,dx);
        angles.push_back(t3);
        return true;
      }

      bool  DoorBaseCollisionCost::findCircleLineSegmentIntersection(const std::vector<double> &p1, const std::vector<double> &p2, const double &x_r, const double &y_r, const double &radius, std::vector<std::vector<double> > intersection_points)
      {
        double eps = 1e-5;
        double dx = p2[0]-p1[0];
        double dy = p2[1]-p1[1];
        double theta = atan2(dy,dx);
        double cost = cos(theta);
        double sint = sin(theta);
        double d = sqrt(dx*dx+dy*dy);

        if(d < eps)
          return false;

        std::vector<double> int_pt;
        int_pt.resize(2);

        double a = pow(d,2);
        double b = -2*x_r*d*cost + 2*p1[0]*d*cost -2*y_r*d*sint + 2*p1[1]*d*sint;
        double c = pow(p1[0],2) + pow(x_r,2) - 2*p1[0]*x_r + pow(p1[1],2) + pow(y_r,2) - 2*p1[1]*y_r - pow(radius,2);

        double disc = pow(b,2) - 4*a*c;
        if(disc < 0)
          return false;

        double t1 = (-b + sqrt(disc))/2*a;   
        if(t1 >= 0 && t1 <= 1) // point is inside limits on the line segment
        {
          int_pt[0] = p1[0]+d*cost*t1;
          int_pt[1] = p1[1]+d*sint*t1;
          intersection_points.push_back(int_pt);
        }

        t1 = (-b - sqrt(disc))/2*a;   
        if(t1 >= 0 && t1 <= 1) // point is inside limits on the line segment
        {
          int_pt[0] = p1[0]+d*cost*t1;
          int_pt[1] = p1[1]+d*sint*t1;
          intersection_points.push_back(int_pt);
        }
        return true;
      }

      void  DoorBaseCollisionCost::findCirclePolygonIntersection(const double &center_x, const double &center_y, const double &radius, const std::vector<std::vector<double> > &footprint, std::vector<std::vector<double> > &solution)
      {
        std::vector<std::vector<double> > intersection_points;

        for(int i=0; i < (int) footprint.size(); i++)
        {
          int p1 = i;
          int p2 = (i+1)%((int) footprint.size());      
          intersection_points.resize(0);
          if (findCircleLineSegmentIntersection(footprint[p1],footprint[p2],center_x,center_y,radius,intersection_points))
          {
            for(int j=0; j< (int) intersection_points.size(); j++)
            {
              solution.push_back(intersection_points[j]);
            }
          }
        }
      }


      void  DoorBaseCollisionCost::freeAngleRange(const std::vector<std::vector<double> > &footprint, const double &door_length,  const double &door_thickness, const double &pivot_length, const double &max_radius, double &min_angle, double &max_angle)
      {
        std::vector<double> free_angles;
        std::vector<double> angles;

        for(int i=0; i < (int) footprint.size(); i++)
        {
          angles.resize(0);
          if(findAngleLimits(door_length,door_thickness,pivot_length,max_radius,footprint[i],angles))
          {
            for(int j=0; j < (int) angles.size(); j++)
            {
               if(!isnan(angles[j]))
              free_angles.push_back(angles[j]);
            }
          }
        }
        std::vector<std::vector<double> > solution;
        findCirclePolygonIntersection(0,0,max_radius,footprint, solution);
        for(int i=0; i < (int) solution.size(); i++)
        {
          angles.resize(0);
          if(findAngleLimits(door_length,door_thickness,pivot_length,max_radius,footprint[i],angles))
          {
            for(int j=0; j < (int) angles.size(); j++)
            {
               if(!isnan(angles[j]))
                  free_angles.push_back(angles[j]);
            }
          }
        }

        if(free_angles.size() < 1)
        {
          min_angle = M_PI/2.0;
          max_angle = 0.0;
          return;
        }
        std::sort(free_angles.begin(),free_angles.end());
        if(free_angles.size() > 1)
        {
          min_angle = free_angles[0];
          max_angle = free_angles[free_angles.size()-1];
        }
        else
        {
          min_angle = free_angles[0];
          max_angle = free_angles[0];
        }
        return;
      }

      void  DoorBaseCollisionCost::getValidDoorAngles(const std::vector<std::vector<double> > &footprint, 
                                                      const std::vector<double> &robot_global_pose, 
                                                      const std::vector<double> &door_global_pose, 
                                                      const std::vector<double> &robot_shoulder_position, 
                                                      const std::vector<double> &door_handle_pose, 
                                                      const double &door_length, const double &door_thickness, const double &pivot_length, 
                                                      const double &min_workspace_angle, const double &max_workspace_angle, 
                                                      const double &min_workspace_radius, const double &max_workspace_radius,
                                                      const double &delta_angle,
                                                      std::vector<int> &valid_angles, 
                                                      std::vector<int> &valid_cost)
      {
        // local door frame is already defined using the door message
        // transform robot footprint into global frame and then into local door frame
        FILE* fp = fopen("doordata.m","wt");
        std::vector<std::vector<double> > global_fp;
        std::vector<std::vector<double> > transformed_fp;

        std::vector<double> global_handle;
        std::vector<double> robot_handle;

        double max_radius = sqrt(pow(door_length,2) + pow(door_thickness + pivot_length,2));

        global_fp.resize(footprint.size());
        transformed_fp.resize(footprint.size());

        fprintf(fp,"hinge_global_position = [%f %f];\n",door_global_pose[0],door_global_pose[1]);
        fprintf(fp,"door_length = %f;\n",door_length);
        fprintf(fp,"door_angle = %f;\n",door_global_pose[2]);

        fprintf(fp,"hinge = [");
        fprintf(fp,"0 0];\n");
        fprintf(fp,"\ndoor = [");
        fprintf(fp,"%f %f\n",0.0,-pivot_length);
        fprintf(fp,"%f %f\n",0.0,-pivot_length-door_thickness);
        fprintf(fp,"%f %f\n",door_length,-pivot_length-door_thickness);
        fprintf(fp,"%f %f];\n",door_length,-pivot_length);
        fprintf(fp,"handle = [ %f, %f];",door_handle_pose[0],door_handle_pose[1]);

        fprintf(fp,"\npose = [");
        for(int j=0; j <3; j++)
        {
          fprintf(fp,"%f ",robot_global_pose[j]);
        }
        fprintf(fp,"];");
        fprintf(fp,"\nglobal_footprint = [");

        for(int i=0; i < (int) footprint.size(); i++)
        {
          transform2D(footprint[i],global_fp[i],robot_global_pose[0],robot_global_pose[1],robot_global_pose[2]);
          fprintf(fp,"%f %f\n",global_fp[i][0],global_fp[i][1]);
          transform2DInverse(global_fp[i],transformed_fp[i],door_global_pose[0],door_global_pose[1],door_global_pose[2]);
        }
        fprintf(fp,"];");

        fprintf(fp,"\nlocal_footprint = [");
        for(int i=0; i < (int) footprint.size(); i++)
        {
          fprintf(fp,"%f %f\n",footprint[i][0],footprint[i][1]);
        }
        fprintf(fp,"];\n");

        std::vector<double> global_shoulder_position;
        transform2D(robot_shoulder_position,global_shoulder_position,robot_global_pose[0],robot_global_pose[1],robot_global_pose[2]);
        fprintf(fp,"\nshoulder_pose = [%f %f];\n",global_shoulder_position[0],global_shoulder_position[1]);
        double min_angle;
        double max_angle;
        freeAngleRange(transformed_fp,door_length,door_thickness,pivot_length,max_radius,min_angle,max_angle);
#ifdef DEBUG
        printf("Free angles are: %f, %f\n",min_angle,max_angle);
#endif
        fprintf(fp,"\nangle_costs = [");
        if(min_angle > 0)
        {
          if(min_angle > M_PI/2.0)
            min_angle = M_PI/2.0;
          int num_intervals = (int) (min_angle/delta_angle);
          for(int i=0; i<num_intervals; i++)
          {
             global_handle.resize(0);
             robot_handle.resize(0);
            double new_angle = i * delta_angle;
            transform2D(door_handle_pose,global_handle,door_global_pose[0],door_global_pose[1],door_global_pose[2]+new_angle);
            transform2DInverse(global_handle,robot_handle,robot_global_pose[0],robot_global_pose[1],robot_global_pose[2]);
            double cost = findWorkspaceCost(robot_shoulder_position,robot_handle,min_workspace_angle,max_workspace_angle,min_workspace_radius,max_workspace_radius);
            if(cost < MAX_COST)
            {
               valid_angles.push_back((int)(new_angle*180.0/M_PI));
               valid_cost.push_back((int) cost);
            }
          }
        }

        if(max_angle < M_PI/2.0)
        {
          if(max_angle < 0)
            max_angle = 0;
          int num_intervals = (int) ((M_PI/2 - max_angle)/delta_angle);
#ifdef DEBUG
          printf("Num intervals: %d, max angle: %f\n",num_intervals,max_angle);
#endif
          for(int i=0; i<num_intervals+1; i++)
          {
             global_handle.clear();
             robot_handle.clear();
            double new_angle = M_PI/2 - i * delta_angle;
            transform2D(door_handle_pose,global_handle,door_global_pose[0],door_global_pose[1],door_global_pose[2]+new_angle);
            transform2DInverse(global_handle,robot_handle,robot_global_pose[0],robot_global_pose[1],robot_global_pose[2]);
            double cost = findWorkspaceCost(robot_shoulder_position,robot_handle,min_workspace_angle,max_workspace_angle,min_workspace_radius,max_workspace_radius);

#ifdef DEBUG
            printf("angle: %f, cost: %f\n",new_angle,cost);
#endif
            if(cost < MAX_COST)
            {
              valid_angles.push_back((int)(new_angle*180.0/M_PI));
              valid_cost.push_back((int)cost);
              fprintf(fp,"%f %f\n", new_angle,cost);
            }
          }
        }
        fprintf(fp,"];");
        fclose(fp);
      }
}

/*using namespace door_base_collision_cost;

int main(int argc, char *argv[])
{
  std::vector<std::vector<double> > footprint;
  std::vector<double> robot_global_pose;
  std::vector<double> door_global_pose;
  std::vector<double> robot_shoulder_position;
  std::vector<double> door_handle_pose;
  std::vector<int> valid_angles;
  std::vector<int> valid_cost;

  double robot_half_width = 0.63/2.0;
  double door_thickness = 0.05;
  double pivot_length = 0.05;
  double door_length = 0.9;

  double min_workspace_radius = 0.2;
  double max_workspace_radius = 1.5;

  double max_workspace_angle = M_PI/2.0;
  double min_workspace_angle = -3*M_PI/2.0;

  double delta_angle = 0.1;

  double x = 0.5;
  double y = -0.5;
  double theta = M_PI/2.0;

  if(argc > 1)
     x = atof(argv[1]);
  if(argc > 2)
     y = atof(argv[2]);
  if(argc > 3)
     theta = atof(argv[3]);


  printf("x: %f, y:%f, theta:%f\n",x,y,theta);
  double shoulder_position_x = -0.2;
  double shoulder_position_y = -0.2;


  footprint.resize(4);
  footprint[0].resize(2);
  footprint[0][0] = robot_half_width;
  footprint[0][1] = robot_half_width;

  footprint[1].resize(2);
  footprint[1][0] = -robot_half_width;
  footprint[1][1] = robot_half_width;

  footprint[2].resize(2);
  footprint[2][0] = -robot_half_width;
  footprint[2][1] = -robot_half_width;

  footprint[3].resize(2);
  footprint[3][0] = robot_half_width;
  footprint[3][1] = -robot_half_width;

  robot_global_pose.resize(3);
  robot_global_pose[0] = x;
  robot_global_pose[1] = y;
  robot_global_pose[2] = theta;

  door_global_pose.resize(3);
  door_global_pose[0] = 0.0;
  door_global_pose[1] = 0.0;
  door_global_pose[2] = 0.0;

  robot_shoulder_position.resize(2);
  robot_shoulder_position[0] = shoulder_position_x;
  robot_shoulder_position[1] = shoulder_position_y;

  door_handle_pose.resize(2);
  door_handle_pose[0] = 0.75;
  door_handle_pose[1] = -0.1;

  DoorBaseCollisionCost db;
  db.getValidDoorAngles(footprint, robot_global_pose, door_global_pose, robot_shoulder_position, door_handle_pose, door_length, door_thickness, pivot_length, min_workspace_angle, max_workspace_angle, min_workspace_radius, max_workspace_radius, delta_angle, valid_angles, valid_cost);
  
  for(int i=0; i < (int) valid_angles.size(); i++)
  {
    printf("Angle: %f, Cost: %f\n",valid_angles[i],valid_cost[i]);
  }

  return(0);
}
*/
