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

#include <generic_controllers/controller.h>
#include <generic_controllers/joint_position_controller.h>
#include <generic_controllers/joint_velocity_controller.h>
#include <generic_controllers/joint_effort_controller.h>

// Services
#include <pr2_controllers/SetBaseCommand.h>
#include <pr2_controllers/GetBaseCommand.h>

#include <libTF/Pose3D.h>
#include <urdf/URDF.h>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

#include <std_msgs/RobotBase2DOdom.h>

namespace controller
{

  /*! \struct
    \brief This class holds information for a joint control parameter structure.
   */
  typedef struct
  {
      double p_gain; /** P gain */

      double i_gain; /** I gain */

      double d_gain; /** D gain */

      double windup; /** windup protection value */

      std::string joint_name; /** joint name */

      std::string control_type; /** control type */

  }JointControlParam;

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

    libTF::Pose3D::Vector pos_; /** position of the link*/

    std::string name_; /** name of joint corresponding to the link */

    JointVelocityController controller_; /** controller for the link */

    mechanism::Joint *joint_; /** pointer to joint in Robot structure corresponding to link */

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
    void init(std::vector<JointControlParam> jcp, mechanism::Robot *robot);
    void initXml(mechanism::Robot *robot, TiXmlElement *config);


    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param double pos Position command to issue
     */
    void setCommand(libTF::Pose3D::Vector cmd_vel);


    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    libTF::Pose3D::Vector getCommand();


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
     * \brief Set the publish count (number of update ticks between odometry message publishing).
     */
    void setPublishCount(int publish_count);


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
     * \brief number of update ticks to wait before publishing ROS odom message
     * defaults to 10.
     */
    int odom_publish_count_;


    /*!
     * \brief local gain used for speed control of the caster (to achieve resultant position control)
     */
    double kp_speed_;


    /*!
     * \brief Robot representation
     */
    mechanism::Robot* robot_;


    /*!
     * \brief compute 2D velocity of a point on a rigid body given a 2D input velocity
     * \param pos - Vector (see libTF for more information)
     * \param vel - Vector, vel.x and vel.y represent translational velocities in X and Y directions, vel.z represents rotational(angular velocity)
     * \return point 2D velocity with .z component set to zero.
     */
    libTF::Pose3D::Vector computePointVelocity2D(const libTF::Pose3D::Vector& pos, const  libTF::Pose3D::Vector& vel);


    /*!
     * \brief update the individual joint controllers
     */
    void updateJointControllers();


    /*!
     * \brief speed command vector used internally
     */
    libTF::Pose3D::Vector cmd_vel_;


    /*!
     * \brief Input speed command vector.
     */
    libTF::Pose3D::Vector cmd_vel_t_;


    /*!
     * \brief Position of the robot computed by odometry.
     */
    libTF::Pose3D::Vector base_odom_position_;


    /*!
     * \brief Speed of the robot computed by odometry.
     */
    libTF::Pose3D::Vector base_odom_velocity_;


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


    std::vector<libTF::Pose3D::Vector> base_wheels_position_; /** vector of current wheel positions */


    /*!
     * \brief get the joint positions and speeds and set them in steer_angle_actual_ and wheel_speed_actual_
     */
    void getJointValues();


    std::vector<double> steer_angle_actual_; /** vector of actual caster steer angles */


    std::vector<double> wheel_speed_actual_; /** vector of actual wheel speeds */


    /*!
     * \brief std_msgs representation of an odometry message
     */
    std_msgs::RobotBase2DOdom odom_msg_;


    double last_time_; /** time corresponding to when update was last called */


    int odom_publish_counter_; /** counter - when this exceeds odom_publish_count_, the odomeetry message will be published on ROS */

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

    void initXml(mechanism::Robot *robot, TiXmlElement *config);

    // Services
    bool setCommand(pr2_controllers::SetBaseCommand::request &req,
                    pr2_controllers::SetBaseCommand::response &resp);

    bool getCommand(pr2_controllers::GetBaseCommand::request &req,
                    pr2_controllers::GetBaseCommand::response &resp);

    private:

    BaseController *c_;

  };

    /** \brief A namespace ostream overload for displaying parameters */
  std::ostream & operator<<(std::ostream& mystream, const controller::BaseParam &bp);

}


