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

#include <iostream> 

/** ROS **/
#include <ros/node.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>

/** TF **/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

/** Messages needed for trajectory control and collision map**/
#include <manipulation_msgs/JointTraj.h>
#include <manipulation_msgs/JointTrajPoint.h>
#include <mapping_msgs/CollisionMap.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <pr2_mechanism_msgs/MechanismState.h>
#include <sensor_msgs/JointState.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

// MPGlue and sbpl headers
#include <mpglue/sbpl_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/plan.h>
#include <sfl/util/strutil.hpp>
#include <sbpl/headers.h>

#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/Pose2D.h>
#include <door_msgs/Door.h>
#include <door_msgs/DoorCmd.h>
#include <door_functions/door_functions.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>

#include <trajectory/trajectory.h>

typedef struct
{
    door_msgs::Door door;
    double door_thickness;
    double arm_min_workspace_angle;
    double arm_max_workspace_angle;
    double arm_min_workspace_radius;
    double arm_max_workspace_radius;
    double door_angle_discretization_interval;
    geometry_msgs::Point32 shoulder;
}DoorEnvProperties;

class SBPLDoorPlanner : public robot_actions::Action<door_msgs::DoorCmd, door_msgs::Door>
{
  public:

  SBPLDoorPlanner();
	
  virtual ~SBPLDoorPlanner();
      
  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue 
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
  virtual robot_actions::ResultStatus execute(const door_msgs::DoorCmd& goal, door_msgs::Door& feedback);

  private:

  ros::NodeHandle ros_node_;
  ros::Subscriber joints_subscriber_;
  ros::Publisher display_path_pub_;
  ros::Publisher start_footprint_pub_;
  ros::Publisher goal_footprint_pub_;
  ros::Publisher robot_footprint_pub_;
  ros::Publisher global_plan_pub_;
  ros::Publisher door_frame_pub_;
  ros::Publisher door_pub_;
  ros::Publisher viz_markers_;
	ros::Publisher viz_marker_array_pub_;
  ros::Publisher pr2_ik_pub_;
  ros::Publisher base_control_pub_;
  tf::TransformListener tf_;
  sensor_msgs::JointState joint_states_;
  motion_planning_msgs::KinematicPath robot_path_;

  void jointsCallback(const sensor_msgs::JointStateConstPtr &joint_states);

//   tf::TransformListener &tf_;  
  boost::shared_ptr<mpglue::CostmapAccessor> cm_access_;
  boost::shared_ptr<mpglue::IndexTransform> cm_index_;
  boost::shared_ptr<mpglue::SBPLEnvironment> env_;
  boost::shared_ptr<mpglue::SBPLPlannerWrap> pWrap_;

  double planner_time_limit_; /* The amount of time given to the planner to find a plan */

  double allocated_time_;

  bool use_cost_map_;

  bool forward_search_;

  std::string planner_type_;

  std::string plan_stats_file_;
  
  costmap_2d::Costmap2DROS *cost_map_ros_; /**< manages the cost map for us */

  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
  
  /** arm planner variables */
  int num_joints_;
  std::vector<std::string> joint_names_;
  motion_planning_msgs::KinematicPath arm_path_;
	
  friend struct cm_getter;
  struct cm_getter: public mpglue::costmap_2d_getter {
      cm_getter(SBPLDoorPlanner * spn): spn_(spn) {}
      virtual costmap_2d::Costmap2D * operator () () { return &spn_->cost_map_; }
      SBPLDoorPlanner * spn_;
  };
  cm_getter cm_getter_;	                   /**< for mpglue to get at our costmap instance */
  
  bool initializePlannerAndEnvironment(const door_msgs::Door &door);
  
  bool makePlan(const pr2_robot_actions::Pose2D &start, const pr2_robot_actions::Pose2D &goal, manipulation_msgs::JointTraj &path);

  std::vector<geometry_msgs::Point> footprint_;

  boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */

  DoorEnvProperties door_env_;

  std::string global_frame_, robot_base_frame_;

  bool updateGlobalPose();

  pr2_robot_actions::Pose2D getPose2D(const tf::Stamped<tf::Pose> &pose);

  pr2_robot_actions::Pose2D global_pose_2D_;

  pr2_robot_actions::Pose2D goal_;

  tf::Stamped<tf::Pose> global_pose_;

  bool removeDoor();

  void publishPath(const manipulation_msgs::JointTraj &path, const ros::Publisher &pub, double r, double g, double b, double a);

  bool computeOrientedFootprint(const pr2_robot_actions::Pose2D &position, const std::vector<geometry_msgs::Point>& footprint_spec, std::vector<geometry_msgs::Point>& oriented_footprint);

  bool clearRobotFootprint(costmap_2d::Costmap2D& cost_map);

  double inflation_radius_;

  double inscribed_radius_;

  void publishFootprint(const pr2_robot_actions::Pose2D &position, const ros::Publisher &pub, double r, double g, double b);

  void publishDoor(const door_msgs::Door &door_in, const double &angle);

  bool animate_;

  double handle_hinge_distance_;

  void animate(const manipulation_msgs::JointTraj &path);

  /**
   * @brief  Returns the global handle position of the door given a door message and a local angle (in the frame of the door)
   * @param door The door message to use
   * @param local_angle The local angle through which the door has rotated (in radians). 0 represents a closed door and (+/-)M_PI/2.0 represents 
   * @return The tf::Stamped pose representing the pose of the handle
   */
  tf::Stamped<tf::Pose> getGlobalHandlePosition(const door_msgs::Door &door, const double &local_angle);

  double getHandleHingeDistance(const door_msgs::Door &door);

  std::string base_control_topic_name_;

  std::string arm_control_topic_name_;

  void dispatchControl(const manipulation_msgs::JointTraj &path, const door_msgs::Door &door);

  double distance_goal_;

  double controller_frequency_;

  double animate_frequency_;

  void publishGripper(const double &angle);

  void processPlan(const manipulation_msgs::JointTraj &path, manipulation_msgs::JointTraj &return_path);

  bool do_control_;

  bool checkArmDoorCollide(const manipulation_msgs::JointTrajPoint &waypoint);

  bool doLineSegsIntersect(geometry_msgs::Point32 a, geometry_msgs::Point32 b, geometry_msgs::Point32 c, geometry_msgs::Point32 d);

  void printPoint(std::string name, geometry_msgs::Point32 point);
	
  /**
   * @brief  Updates the kinematic path visualization that is then published to display_kinematic_path
   * @param frame_id the frame of the path
   * @param joints is a vector of strings representing the joint names in the path 
   * @param joint_vals is a 2D vector of joint waypoints with size: {number of waypoints} x {number of joints}  (joint angles in radians)
   * @param planar_links is a vector of strings representing the planar link names in the path
   * @param planar_vals is a 2D vector of planar link waypoints with size: {number of waypoints} x {number of planar links x 3 {x,y,theta}}
   */
  void updatePathVizualization(std::string frame_id, std::vector<std::string> &joints, std::vector<std::vector<double> > &joint_vals,
                               std::vector<std::string> &planar_links, std::vector<std::vector<double> > &planar_vals);
	
  void getCurrentJointAngles(const std::vector <std::string> &joint_names, std::vector <double> *joint_angles);

  double gripper_palm_wrist_distance_;

  int door_open_direction_;

  manipulation_msgs::JointTraj prunePlan(const manipulation_msgs::JointTraj &path, 
                                                          const double &start_door_angle, 
                                                          const double &end_door_angle, 
                                                          const double &tolerance,
                                                          const bool &impose_monotonic);

  bool scalePlan(const manipulation_msgs::JointTraj &traj, const double dT, manipulation_msgs::JointTraj &traj_out);

  bool createTrajectoryPointsVectorFromMsg(const manipulation_msgs::JointTraj &new_traj, std::vector<trajectory::Trajectory::TPoint> &tp);

  bool createMsgFromTrajectory(const std::vector<trajectory::Trajectory::TPoint> &tp, manipulation_msgs::JointTraj &new_traj);

  bool createTrajectoryFromMsg(const manipulation_msgs::JointTraj &new_traj, trajectory::Trajectory &return_trajectory);

  std::string trajectory_type_;

  std::vector<double> velocity_limits_;
	
	void displayExpandedStates();

  int door_index_;

  void printPlan(const std::string name, manipulation_msgs::JointTraj &traj_out);

};
