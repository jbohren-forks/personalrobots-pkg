/*********************************************************************
 *
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
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <highlevel_controllers/move_base.hh>

using namespace costmap_2d;

namespace ros {
  namespace highlevel_controllers {
    /**
     * @brief Specialization for the SBPL planner
     */
    class MoveBaseFollow: public MoveBase {
      public:
        MoveBaseFollow();

      private:

        /**
         * @brief Builds a plan from current state to goal state
         */
        virtual bool makePlan();
        virtual bool dispatchCommands();

    };


    MoveBaseFollow::MoveBaseFollow()
      : MoveBase()
    {
      initialize();
    }

    bool MoveBaseFollow::makePlan(){
      // Lock the state message and obtain current goal
      stateMsg.lock();
      double goalX = stateMsg.goal.x, goalY = stateMsg.goal.y;
      stateMsg.unlock();

      // Simply pass on the goal as the plan for the controller
      std::list<deprecated_msgs::Pose2DFloat32> newPlan;

      //This is a hack for now to add the goal.
      deprecated_msgs::Pose2DFloat32 goalstep;
      goalstep.x = goalX;
      goalstep.y = goalY;
      newPlan.push_back(goalstep);

      updatePlan(newPlan);

      return true;

    }

    /**
     * Note that we have tried doing collision checks on the plan, and replanning in that case at the global level. With ARA*
     * at least, this leads to poor performance since planning ks slow (seconds) and thus the robot stops alot. If we leave the
     * velocity controller flexibility to work around the path in collision then that seems to work better. Note that this is still
     * sensitive to the goal (exit point of the path in the window) being in collision which would require an alternate metric
     * to allow more flexibility to get near the goal - essentially treating the goal as a waypoint. 
     */
    bool MoveBaseFollow::dispatchCommands(){
      // First criteria is that we have had a sufficiently recent sensor update to trust perception and that we have a valid plan. This latter
      // case is important since we can end up with an active controller that becomes invalid through the planner looking ahead. 
      // We want to be able to stop the robot in that case
      bool planOk = checkWatchDog() && isValid();
      robot_msgs::PoseDot cmdVel; // Commanded velocities      

      // if we have achieved all our waypoints but have yet to achieve the goal, then we know that we wish to accomplish our desired
      // orientation
      if(planOk && plan_.empty()){
        double uselessPitch, uselessRoll, yaw;
        global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
        ROS_DEBUG("Moving to desired goal orientation\n");
        cmdVel.vel.vx = 0;
        cmdVel.vel.vy = 0;
        cmdVel.ang_vel.vz =  stateMsg.goal.th - yaw;
        cmdVel.ang_vel.vz = cmdVel.ang_vel.vz >= 0.0 ? cmdVel.ang_vel.vz + .4 : cmdVel.ang_vel.vz - .4;
      }
      else {
        // The plan is bogus if it is empty
        if(planOk && plan_.empty()){
          planOk = false;
          ROS_DEBUG("No path points in local window.\n");
        }

        // Set current velocities from odometry
        robot_msgs::PoseDot currentVel;
        currentVel.vel.vx = base_odom_.vel.x;
        currentVel.vel.vy = base_odom_.vel.y;
        currentVel.ang_vel.vz = base_odom_.vel.th;

        ros::Time start = ros::Time::now();
        // Create a window onto the global cost map for the velocity controller
        std::list<deprecated_msgs::Pose2DFloat32> localPlan; // Capture local plan for display

        lock();
        // Aggregate buffered observations across all sources. Must be thread safe
        std::vector<costmap_2d::Observation> observations;
        baseScanBuffer_->get_observations(observations);
        tiltScanBuffer_->get_observations(observations);
        lowObstacleBuffer_->get_observations(observations);
        stereoCloudBuffer_->get_observations(observations);
        unlock();

        if(planOk && !controller_->computeVelocityCommands(plan_, global_pose_, currentVel, cmdVel, localPlan, observations)){
          ROS_DEBUG("Velocity Controller could not find a valid trajectory.\n");
          planOk = false;
        }
        ROS_DEBUG("Cycle Time: %.3f\n", (ros::Time::now() - start).toSec());

        if(!planOk){
          // Zero out the velocities
          cmdVel.vel.vx = 0;
          cmdVel.vel.vy = 0;
          cmdVel.ang_vel.vz = 0;
        }
        else {
          publishPath(false, localPlan);
        }
      }

      publish("cmd_vel", cmdVel);
      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      publishFootprint(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

      //publish a point that the head can track
      double ptx, pty;
      controller_->getLocalGoal(ptx, pty);
      robot_msgs::PointStamped target_point;
      target_point.point.x = ptx;
      target_point.point.y = pty;
      target_point.point.z = 1;
      target_point.header.stamp = ros::Time::now();
      target_point.header.frame_id = global_frame_;
      publish("head_controller/head_track_point", target_point);
      return planOk;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ros::highlevel_controllers::MoveBaseFollow node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }



  return(0);
}
