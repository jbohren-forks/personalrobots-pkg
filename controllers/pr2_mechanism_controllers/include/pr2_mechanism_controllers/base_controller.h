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

#pragma once

#include <ros/node.h>

#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <robot_mechanism_controllers/joint_effort_controller.h>

// Services
#include <pr2_mechanism_controllers/SetBaseCommand.h>
#include <pr2_mechanism_controllers/GetBaseCommand.h>

#include <libTF/Pose3D.h>
#include <urdf/URDF.h>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>

#include <rosTF/rosTF.h>

#include <pthread.h>

namespace controller
{
  #define KP_SPEED_DEFAULT 100
  #define DEFAULT_WHEEL_RADIUS 0.079

  /*! \class
    \brief This class holds local information for the links in the base (wheels and casters). It includes position, name,
    controller, pointer to the corresponding joint in the Robot structure, pointer to parent and a local ID.
  */
  class BaseParam
  {
    public:

    friend std::ostream & operator<<(std::ostream& mystream, const controller::BaseParam &bp);

    BaseParam():direction_multiplier_(1){};

    ~BaseParam(){}

    libTF::Vector pos_; /** position of the link*/

    std::string name_; /** name of joint corresponding to the link */

    JointVelocityController controller_; /** controller for the link */

    mechanism::JointState *joint_state_; /** pointer to joint in Robot structure corresponding to link */

    BaseParam *parent_; /** pointer to parent corresponding to link */

    int local_id_; /** local id number */

    int direction_multiplier_;
  };

  class BaseController : public Controller
  {
    public:

    /*!
     * \brief Default Constructor of the JointController class.
     *
     */
    BaseController();


    /*!
     * \brief Destructor of the JointController class.
     */
    ~BaseController();


    /*!
     * \brief Functional way to initialize limits and gains.
     *
     */
    void init(std::vector<JointControlParam> jcp, mechanism::RobotState *robot);
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param double pos Position command to issue
     */
    void setCommand(libTF::Vector cmd_vel);


    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    libTF::Vector getCommand();


    /*!
     * \brief (a) Updates commands to caster and wheels.
     *        (b) Computes odometry
     *        (c) Publishes odometry
     *  Should be called at regular intervals
     */
    virtual void update();


    /*!
     * \brief struct to represent caster parameters locally.
     */
    std::vector<BaseParam> base_casters_;


    /*!
     * \brief struct to represent wheel parameters locally.
     */
    std::vector<BaseParam> base_wheels_;


    /*!
     * \brief mutex lock for setting and getting commands
     */
    pthread_mutex_t base_controller_lock_;

    /*!
     * \brief URDF representation of the robot model
     */
    robot_desc::URDF urdf_model_;


    /*!
     * \brief returns odometry data
     */
    void getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw);

    /*!
     * \brief set odometry message
     */
    void setOdomMessage(std_msgs::RobotBase2DOdom &odom_msg_);

    void computeWheelSpeeds();

    void setWheelSpeeds();

    void computeCasterSteer();

    void setCasterSteer();

    void computeCommands();

    void setCommands();

    NEWMAT::Matrix iterativeLeastSquares(NEWMAT::Matrix A, NEWMAT::Matrix b, std::string weight_type, int max_iter);

    NEWMAT::Matrix findWeightMatrix(NEWMAT::Matrix residual, std::string weight_type);

    int ils_max_iterations_;

    std::string ils_weight_type_;

    private:

    /*!
     * \brief number of wheels
     */
    int num_wheels_;


    /*!
     * \brief number of casters
     */
    int num_casters_;

    /*!
     * \brief local gain used for speed control of the caster (to achieve resultant position control)
     */
    double kp_speed_;


    /*!
     * \brief Robot representation
     */
    mechanism::RobotState* robot_state_;


    /*!
     * \brief compute 2D velocity of a point on a rigid body given a 2D input velocity
     * \param pos - Vector (see libTF for more information)
     * \param vel - Vector, vel.x and vel.y represent translational velocities in X and Y directions, vel.z represents rotational(angular velocity)
     * \return point 2D velocity with .z component set to zero.
     */
    libTF::Vector computePointVelocity2D(const libTF::Vector& pos, const  libTF::Vector& vel);

    /*!
     * \brief update the individual joint controllers
     */
    void updateJointControllers();


    /*!
     * \brief speed command vector used internally
     */
    libTF::Vector cmd_vel_;


    /*!
     * \brief Input speed command vector.
     */
    libTF::Vector cmd_vel_t_;


    /*!
     * \brief Position of the robot computed by odometry.
     */
    libTF::Vector base_odom_position_;


    /*!
     * \brief Speed of the robot computed by odometry.
     */
    libTF::Vector base_odom_velocity_;


    /*!
     * \brief Computed the desired steer angle for the caster.
     */
    void computeAndSetCasterSteer();


    /*!
     * \brief Computed the desired wheel speeds.
     */
    void computeAndSetWheelSpeeds();


    double wheel_radius_; /** radius of the wheel (filled in from urdf robot model) */


    std::vector<double> steer_velocity_desired_; /** vector of desired caster steer speeds */

    std::vector<double> wheel_speed_cmd_; /** vector of desired wheel speeds */


    /*!
     * \brief compute the speed of the base for odometry calculations
     */
    void computeBaseVelocity();


    /*!
     * \brief compute the odometry
     */
    void computeOdometry(double);


    /*!
     * \brief pseudo-inverse computation for NEWMAT
     */
    NEWMAT::Matrix pseudoInverse(const NEWMAT::Matrix M);


    /*!
     * \brief compute the wheel positions and set them in base_wheels_position_
     */
    void computeWheelPositions();


    std::vector<libTF::Vector> base_wheels_position_; /** vector of current wheel positions */


    /*!
     * \brief get the joint positions and speeds and set them in steer_angle_actual_ and wheel_speed_actual_
     */
    void getJointValues();


    std::vector<double> steer_angle_actual_; /** vector of actual caster steer angles */

    std::vector<double> wheel_speed_actual_; /** vector of actual wheel speeds */

    double last_time_; /** time corresponding to when update was last called */


    int odom_publish_counter_; /** counter - when this exceeds odom_publish_count_, the odometry message will be published on ROS */

    double caster_steer_vel_gain_;

    double cmd_received_timestamp_;

    double timeout_;

    double max_x_dot_;

    double max_y_dot_;

    double max_yaw_dot_;
  };

  class BaseControllerNode : public Controller
  {
    public:
    /*!
     * \brief Default Constructor
     *
     */
    BaseControllerNode();

    /*!
     * \brief Destructor
     */
    ~BaseControllerNode();

    void update();

    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
    void getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw);

    // Services
    bool setCommand(pr2_mechanism_controllers::SetBaseCommand::request &req,
                    pr2_mechanism_controllers::SetBaseCommand::response &resp);

    bool getCommand(pr2_mechanism_controllers::GetBaseCommand::request &req,
                    pr2_mechanism_controllers::GetBaseCommand::response &resp);

    void setCommand(double vx, double vy, double vw);

    BaseController *c_;

    private:

    /*!
     * \brief deal with cmd_vel command from 2dnav stack
     */
    void CmdBaseVelReceived();
    std_msgs::BaseVel baseVelMsg;

    /*!
     * \brief mutex lock for setting and getting ros messages
     */
    ros::thread::mutex ros_lock_;

    /*!
     * \brief frame transform server for publishing base odometry frame
     */
    private: rosTFServer *tfs;

    /*!
     * \brief Set the publish count (number of update ticks between odometry message publishing).
     */
    void setPublishCount(int publish_count); //FIXME: use time rather than count

    /*!
     * \brief std_msgs representation of an odometry message
     */
    std_msgs::RobotBase2DOdom odom_msg_;

    /*!
     * \brief number of update ticks to wait before publishing ROS odom message
     * defaults to 10.
     */
    int odom_publish_count_; //FIXME: use time rather than count


    int odom_publish_counter_; /** counter - when this exceeds odom_publish_count_, the odomeetry message will be published on ROS */ //FIXME: use time rather than count
  };

    /** \brief A namespace ostream overload for displaying parameters */
  std::ostream & operator<<(std::ostream& mystream, const controller::BaseParam &bp);

    /*
     * \brief pointer to ros node
     */
    ros::node *node;
    /*
     * \brief save service name prefix for unadvertise on exit
     */
    std::string service_prefix;
}


