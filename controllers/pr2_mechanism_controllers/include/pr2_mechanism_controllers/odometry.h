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
#include <realtime_tools/realtime_publisher.h>

#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <ros/node.h>

#include <deprecated_msgs/RobotBase2DOdom.h>
#include <angles/angles.h>

typedef Eigen::Matrix<float, 3, 1> OdomMatrix3x1;
typedef Eigen::Matrix<float, 16, 1> OdomMatrix16x1;
typedef Eigen::Matrix<float, 16, 3> OdomMatrix16x3;
typedef Eigen::Matrix<float, 16, 16> OdomMatrix16x16;

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

      ~Odometry();

      bool initXml(mechanism::RobotState *robot_state, TiXmlElement *config);

      bool starting();

      void update();

      void publish();

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     private:

      void init();

      robot_msgs::PoseDot pointVel2D(const robot_msgs::Point& pos, const robot_msgs::PoseDot& vel);

      void updateOdometry();

      void getOdometryMessage(pr2_msgs::Odometry &msg);

      void getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vw);

      void getOdometry(robot_msgs::Point &odom, robot_msgs::PoseDot &odom_vel);

      void computeBaseVelocity();

      double getCorrectedWheelSpeed(int index);

      Eigen::MatrixXf iterativeLeastSquares(Eigen::MatrixXf lhs, Eigen::MatrixXf rhs, std::string weight_type, int max_iter);

      Eigen::MatrixXf findWeightMatrix(Eigen::MatrixXf residual, std::string weight_type);

      std::vector<Wheel> wheel_;

      double odometer_distance_;

      double odometer_angle_;

      double last_time_,current_time_;

      int num_wheels_;

//      OdomMatrix16x1 cbv_rhs_, fit_rhs_, fit_residual_, odometry_residual_;

      Eigen::MatrixXf cbv_rhs_, fit_rhs_, fit_residual_, odometry_residual_;

      Eigen::MatrixXf cbv_lhs_, fit_lhs_;
      
      Eigen::MatrixXf cbv_soln_,fit_soln_,  weight_matrix_;

//      OdomMatrix16x3 cbv_lhs_, fit_lhs_;
      
//      OdomMatrix3x1 cbv_soln_,fit_soln_;

//      OdomMatrix16x16 weight_matrix_;

      robot_msgs::Point odom_;

      robot_msgs::PoseDot odom_vel_;

      std::string ils_weight_type_,xml_wheel_name_;

      int ils_max_iterations_;

      double wheel_radius_;

      double odometry_residual_max_;

      std::string odom_frame_;

      mechanism::RobotState *robot_state_;

      double last_publish_time_;

      double expected_publish_time_;

      realtime_tools::RealtimePublisher <pr2_msgs::Odometry>* odometry_publisher_ ;  

      realtime_tools::RealtimePublisher <deprecated_msgs::RobotBase2DOdom>* old_odometry_publisher_ ;  

      realtime_tools::RealtimePublisher <tf::tfMessage>* transform_publisher_ ;  

      void getOldOdometryMessage(deprecated_msgs::RobotBase2DOdom &msg);

   };
}
