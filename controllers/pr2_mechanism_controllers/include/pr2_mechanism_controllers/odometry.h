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

#include <Eigen/Array>
#include <Eigen/SVD>

#include <robot_msgs/PoseDot.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/Vector3.h>
#include <pr2_msgs/Odometry.h>

#include <mechanism_model/robot.h>
#include <mechanism_model/controller.h>


namespace controller
{
   class Wheel
   {
     public:
      
      mechanism::JointState *joint_;

      mechanism::JointState *steer_;

      robot_msgs::Point offset_;

      robot_msgs::Point steer_offset_;

      std::string name_;

      robot_msgs::Point position_;
      
      void initXml(mechanism::RobotState *robot_state, TiXmlElement *config);

      void updatePosition();

   };

   class Odometry : public Controller
   {
     public:

      Odometry();

      ~Odometry(){};

      bool initXml(mechanism::RobotState *robot_state, TiXmlElement *config);

      bool starting();

      void update();

     private:

      void init();

      robot_msgs::PoseDot pointVel2D(const robot_msgs::Point& pos, const robot_msgs::PoseDot& vel);

      void updateOdometry();

      pr2_msgs::Odometry getOdometryMessage();

      void getOdometry(robot_msgs::Point &odom, robot_msgs::PoseDot &odom_vel);

      void computeBaseVelocity();

      double getCorrectedWheelSpeed(int index);

      Eigen::Vector3f solve(Eigen::MatrixXf lhs, Eigen::MatrixXf rhs);

      Eigen::MatrixXf iterativeLeastSquares(Eigen::MatrixXf lhs, Eigen::MatrixXf rhs, std::string weight_type, int max_iter);

      Eigen::MatrixXf findWeightMatrix(Eigen::MatrixXf residual, std::string weight_type);

      std::vector<Wheel> wheel_;

      double odometer_distance_;

      double odometer_angle_;

      double last_time_,current_time_;

      int num_wheels_;

      Eigen::MatrixXf cbv_rhs_,cbv_lhs_,cbv_soln_,odometry_residual_;

      Eigen::MatrixXf weight_matrix_, fit_lhs_, fit_rhs_, fit_residual_, fit_soln_;

      robot_msgs::Point odom_;

      robot_msgs::PoseDot odom_vel_;

      std::string ils_weight_type_,xml_wheel_name_;

      int ils_max_iterations_;

      double wheel_radius_;

      double odometry_residual_max_;

      std::string odom_frame_;

      mechanism::RobotState *robot_state_;
   };
}
