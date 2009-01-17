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
#include <robot_mechanism_controllers/joint_pd_controller.h>

// Services
#include <pr2_mechanism_controllers/SetBaseCommand.h>
#include <pr2_mechanism_controllers/GetBaseCommand.h>
#include <pr2_mechanism_controllers/WheelRadiusMultiplier.h>
#include <pr2_mechanism_controllers/OdometryResiduals.h>

#include <libTF/Pose3D.h>
#include <urdf/URDF.h>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>
#include <pr2_msgs/Odometer.h>
#include <pr2_msgs/Covariance2D.h>

#include <misc_utils/realtime_publisher.h>

#include <tf/tfMessage.h>

#include <pthread.h>
#include <trajectory/trajectory.h>

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

    BaseParam():direction_multiplier_(1),name_(" "),joint_state_(NULL),parent_(NULL){};

    ~BaseParam(){}

    libTF::Vector pos_; /** position of the link*/

    std::string name_; /** name of joint corresponding to the link */

    JointVelocityController velocity_controller_; /** controller for the link */

    JointPositionController position_controller_; /** position controller for the link */

    mechanism::JointState *joint_state_; /** pointer to joint in Robot structure corresponding to link */

    BaseParam *parent_; /** pointer to parent corresponding to link */

    int local_id_; /** local id number */

    int direction_multiplier_;
  };

  /*! \class
    \brief This class inherits from Controller and implements the actual controls.
  */
  class BaseControllerPos : public Controller
  {
    public:

    /*!
     * \brief Default Constructor of the JointController class.
     *
     */
    BaseControllerPos();


    /*!
     * \brief Destructor of the JointController class.
     */
    ~BaseControllerPos();


    /*!
     * \brief Functional way to initialize limits and gains.
     *
     */
    void init(std::vector<JointControlParam> jcp, mechanism::RobotState *robot);
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


    /*!
     * \brief Specify the commanded velocity for the controller using a libTF::Vector data structure
     *
     * \param libTF::Vector cmd_vel with cmd_vel.x and cmd_vel.y specifying the forward and sideways speeds respectively while cmd_vel.z specifies the rotational speed.
     */
    void setCommand(libTF::Vector cmd_vel);


    /*!
     * \brief Get the current command. This is the command that is currently being executed by the controller and may differ from the actual command issued because of the imposition of acceleration constraints.
     */
    libTF::Vector getCommand();


    /*!
     * \brief (a) Updates commands to caster and wheels.
     *         
     *  Called every timestep in realtime
     */
    virtual void update();


    /*!
     * \brief struct to represent caster parameters locally.
     */
//    std::vector<BaseParam> base_casters_;
    BaseParam base_casters_[4];

    /*!
     * \brief struct to represent wheel parameters locally.
     */
//    std::vector<BaseParam> base_wheels_;
    BaseParam base_wheels_[8];

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
     * \param x position estimate from odometry (origin is at the starting point when controller was started)
     * \param y position estimate from odometry (origin is at the starting point when controller was started)
     * \param w position estimate from odometry (origin is at the starting point when controller was started)
     * \param vx velocity estimate from odometry (in the coordinate frame of the base)
     * \param vy velocity estimate from odometry (in the coordinate frame of the base)
     * \param vz velocity estimate from odometry (in the coordinate frame of the base)
     */
    void getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw);

    /*!
     * \brief Fill a structure of the form std_msgs::RobotBase2DOdom with odometry data
     */
    void setOdomMessage(std_msgs::RobotBase2DOdom &odom_msg_);

    /*!
     * \brief compute the desired wheel speeds
     */
    void computeDesiredWheelSpeeds();

    /*!
     * \brief set the desired wheel speeds in the joint speed controllers
     */
    void setDesiredWheelSpeeds();

    /*!
     * \brief compute the desired caster steer
     */
    void computeDesiredCasterSteer(double current_sample_time, double dT);

    /*!
     * \brief set the desired caster steer
     */
    void setDesiredCasterSteer();

    /*!
     * \brief compute the joint commands 
     */
    void computeJointCommands(double current_sample_time, double dT);

    /*!
     * \brief set the joint commands
     */
    void setJointCommands();

    /*!
     * \brief figure out the command for the next time step by interpolating from start to end while staying within rate limits
     * specified by max_rate
     * \param dT - timestep
     */
    libTF::Vector interpolateCommand(double time, bool set);

    /*!
     * \brief iterative least squares implementation to compute odometry. This implementation computes and returns x where A*x = b
     * \param A - newmat matrix of size m x n
     * \param b - newmat matrix of size m x 1
     * \param weight_type - type of weighting used in the iterations, possible choices include BubeLagan, L1norm, fair, Cauchy, Gaussian
     * \param max_iter - maximum number of iterations
     * \return x - the n x 1 result as a NEWMAT matrix
     */
    NEWMAT::Matrix iterativeLeastSquares(NEWMAT::Matrix A, NEWMAT::Matrix b, std::string weight_type, int max_iter);

    /*!
     * \brief returns the weight matrix given the weight type and residual
     * \param residual (a m x 1 NEWMAT matrix)
     * \param weight_type - std::string specification of weight type
     */
    NEWMAT::Matrix findWeightMatrix(NEWMAT::Matrix residual, std::string weight_type);

    int ils_max_iterations_; /** maximum number of iterations for Iterative Least Squares */

    std::string ils_weight_type_; /** std::string specification of weight type used for IRLS*/

    /*!
     * \brief Robot representation
     */
    mechanism::RobotState* robot_state_;

    double wheel_radius_; /** radius of the wheel (filled in from urdf robot model) */

    double wheel_radius_multiplier_front_;

    double wheel_radius_multiplier_rear_;

    trajectory::Trajectory *cmd_vel_trajectory_;

    trajectory::Trajectory::TPoint current_vel_point_;

    std::vector<trajectory::Trajectory::TPoint> cmd_vel_points_;

    libTF::Vector cmd_vel_direction_;

    libTF::Vector cmd_vel_dt_direction_;

    double cmd_vel_magnitude_;

    double sample_time_;

    void setVelocityCmdTrajectory(libTF::Vector new_cmd, libTF::Vector max_rate);

    private:

    std::vector<double> odometry_residuals_vector_;

    bool new_cmd_available_; /** true when new command received by node */

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
     * \brief speed command vector used internally to represent the current commanded speed
     */
    libTF::Vector cmd_vel_;


    /*!
     * \brief Input speed command vector represents the desired speed requested by the node. Note that this may differ from the 
     * current commanded speed due to acceleration limits imposed by the controller. This 
     */
    libTF::Vector cmd_vel_t_;

    /*!
     * \brief Input speed command vector represents the desired speed requested by the node. 
     */
    libTF::Vector desired_vel_;

    /*!
     * \brief Position of the robot computed by odometry.
     */
    libTF::Vector base_odom_position_;


    /*!
     * \brief Speed of the robot computed by odometry.
     */
    libTF::Vector base_odom_velocity_;


    std::vector<double> steer_velocity_desired_; /** vector of desired caster steer speeds */

    std::vector<double> steer_position_desired_; /** vector of desired caster steer positions */

    std::vector<double> steer_torque_desired_; /** vector of desired caster steer positions */

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

    /*!
     * \brief function to add parameter to map so it can be initialized easily from the xml file
     */
    void addParamToMap(std::string key, double *value); 

    std::vector<double> steer_angle_actual_; /** vector of actual caster steer angles */

    std::vector<double> steer_angle_stored_; /** vector of stored caster steer angles */

    std::vector<double> wheel_speed_actual_; /** vector of actual wheel speeds */

    double last_time_; /** time corresponding to when update was last called */

    double caster_steer_vel_gain_; /** gain specifying the amount of help given by the wheels to the caster steer degree of freedom */

    double cmd_received_timestamp_; /** timestamp corresponding to when the command received by the node */

    double timeout_; /** timeout specifying time that the controller waits before setting the current velocity command to zero */

    libTF::Vector max_vel_; /** velocity limits specified externally */

    libTF::Vector max_accel_; /** acceleration limits specified externally */

    double MAX_DT_; /** maximum dT used in computation of interpolated velocity command */

    std::map<std::string, double*> param_map_; /*< map from pointers to the params to string names */

    double odometer_distance_;

    double odometer_angle_;

    double odometry_residual_max_;

    friend class BaseControllerPosNode;
  };

  /*! \class
    \brief This class inherits from Controller and is the ROS Node corresponding to the controller. This is the class that should be instantiated whenever the user wants to spawn a new base controller. 
  */
  class BaseControllerPosNode : public Controller
  {
    public:
    /*!
     * \brief Default Constructor
     *
     */
    BaseControllerPosNode();

    /*!
     * \brief Destructor
     */
    ~BaseControllerPosNode();

    /*!
     * realtime safe update call
     */
    void update();

    /*!
     * \brief initialize the node from a xml config specification
     */
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /*!
     * \brief get back the odometry values from the controller itself
     * \param x position computed by odometry 
     * \param y position computed by odometry
     * \param theta (yaw) computed by odometry
     * \param x velocity computed by odometry (in local frame)
     * \param y velocity computed by odometry (in local frame)
     * \param theta velocity computed by odometry (in local frame)
     */
    void getOdometry(double &x, double &y, double &w, double &vx, double &vy, double &vw);

    // Services
    bool setCommand(pr2_mechanism_controllers::SetBaseCommand::request &req,
                    pr2_mechanism_controllers::SetBaseCommand::response &resp);

    bool getCommand(pr2_mechanism_controllers::GetBaseCommand::request &req,
                    pr2_mechanism_controllers::GetBaseCommand::response &resp);

    bool getWheelRadiusMultiplier(pr2_mechanism_controllers::WheelRadiusMultiplier::request &req,
                                                      pr2_mechanism_controllers::WheelRadiusMultiplier::response &resp);

    bool setWheelRadiusMultiplier(pr2_mechanism_controllers::WheelRadiusMultiplier::request &req,
                                                      pr2_mechanism_controllers::WheelRadiusMultiplier::response &resp);

    /*
     * \brief callback function for setting the desired velocity using a topic 
     */
    void setCommand(double vx, double vy, double vw);

    BaseControllerPos *c_;

    private:

    /*!
     * \brief deal with cmd_vel command from 2dnav stack
     */
    void CmdBaseVelReceived();
    std_msgs::BaseVel baseVelMsg;

    /*!
     * \brief mutex lock for setting and getting ros messages
     */
    boost::mutex ros_lock_;

    /*!
     * \brief std_msgs representation of an odometry message
     */
    std_msgs::RobotBase2DOdom odom_msg_;

    double last_time_message_sent_ ;/** last time odometry message was published */

    double odom_publish_delta_t_; /** time after which odometry message will be published */

    double odom_publish_rate_; /** rate at which odometry message will be published ( = 1/odom_publish_delta_t_)*/
           
    misc_utils::RealtimePublisher <std_msgs::RobotBase2DOdom>* publisher_ ;  //!< Publishes the odometry msg from the update() realtime loop

    misc_utils::RealtimePublisher <tf::tfMessage>* transform_publisher_ ;  //!< Publishes the odom to base transform msg from the update() realtime loop

    misc_utils::RealtimePublisher <pr2_msgs::Odometer>* odometer_publisher_ ;  //!< Publishes the odom to base transform msg from the update() realtime loop

    misc_utils::RealtimePublisher <pr2_msgs::Covariance2D>* covariance_publisher_ ;  //!< Publishes the odom to base transform msg from the update() realtime loop

    misc_utils::RealtimePublisher <pr2_mechanism_controllers::OdometryResiduals>* residuals_publisher_ ;  //!< Publishes the odom to base transform msg from the update() realtime loop

    /*
     * \brief pointer to ros node
     */
    ros::Node *node;
    /*
     * \brief save service name prefix for unadvertise on exit
     */
    std::string service_prefix;

  };

    /** \brief A namespace ostream overload for displaying parameters */
//  std::ostream & operator<<(std::ostream& mystream, const controller::BaseParam &bp);

}


