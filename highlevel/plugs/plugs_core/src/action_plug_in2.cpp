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
 *   * Neither the name of Willow Garage nor the names of its
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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INeco
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <plugs_core/action_plug_in2.h>


namespace plugs_core {

const double READY_TO_INSERT = -0.026;
const double READY_TO_PUSH = -0.01;  //-0.0150;  // -0.0135;  // -0.009;
const double FORCING_SUCCESSFUL = -0.009;

const double MIN_STANDOFF = 0.03;

const char* COMMAND_FRAME = "outlet_pose";
const double SPIRAL_STEP = 0.002;

const double SUCCESS_THRESHOLD = 0.025;
enum {MEASURING, MOVING, INSERTING, FORCING, HOLDING};

const static char* TRACKER_ACTIVATE = "/plug_detector/activate_tracker";


double getOffset(int id)
{
  switch (id)
  {
  case 0: return 1.0;
  case 1: return 0.98948537037195827;
  case 6: return 0.97692542711765129;
  case 39: return 0.98097524550697912;
  case 4: return 0.97561703536950839;
  case 3: return 0.99092973248973848;
  case 38: return 0.95012308281214775;
  case 40: return 0.98361298809350939;
  case 27: return 0.9864272331729832;
  case 26: return 0.98778051326139238;
  case 25: return 0.98898110374305592;
  case 20: return 0.98890562635876034;
  case 21: return 0.9780777627427707;
  default:
    ROS_ERROR("Invalid outlet id: %d", id);
    return 0;
  }
}

void PoseTFToMsg(const tf::Pose &p, robot_msgs::Twist &t)
{
  t.vel.x = p.getOrigin().x();
  t.vel.y = p.getOrigin().y();
  t.vel.z = p.getOrigin().z();
  btMatrix3x3(p.getRotation()).getEulerYPR(t.rot.x, t.rot.y, t.rot.z);
}

void PoseMsgToTF(const robot_msgs::Twist &t, tf::Pose &p)
{
  p.getOrigin()[0] = t.vel.x;
  p.getOrigin()[1] = t.vel.y;
  p.getOrigin()[2] = t.vel.z;
  p.getBasis().setEulerYPR(t.rot.z, t.rot.y, t.rot.x);
}


PlugInAction::PlugInAction(ros::Node& node) :
  robot_actions::Action<std_msgs::Int32, std_msgs::Empty>("plug_in"),
  action_name_("plug_in"),
  node_(node),
  arm_controller_("r_arm_hybrid_controller")
{
  node_.param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);

  if(arm_controller_ == "" )
  {
    ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
    terminate();
    return;
  }

  node_.advertise<manipulation_msgs::TaskFrameFormalism>(arm_controller_ + "/command", 2);

  TF_.reset(new tf::TransformListener(node));
  notifier_.reset(new tf::MessageNotifier<robot_msgs::PoseStamped>(
                    TF_.get(), &node,
                    boost::bind(&PlugInAction::plugMeasurementCB, this, _1),
                    "/plug_detector/plug_pose", "outlet_pose", 100));
  node_.subscribe(arm_controller_ + "/state", c_state_msg_, &PlugInAction::controllerStateCB, this, 2);

  tff_msg_.header.frame_id = "outlet_pose";

  node_.advertise<plugs_core::PlugInState>(action_name_ + "/state", 10);
  node_.advertise<std_msgs::Empty>(TRACKER_ACTIVATE, 1);
};

PlugInAction::~PlugInAction()
{
  node_.unadvertise(TRACKER_ACTIVATE);
};

robot_actions::ResultStatus PlugInAction::execute(const std_msgs::Int32& outlet_id, std_msgs::Empty& feedback)
{
  std_msgs::Empty empty;
  reset();

  enum {APPROACHING, FIRST_TOUCH, SPIRALING, FORCING} state = APPROACHING;
  tf::Pose outlet_pose; // As considered by the mechanism
  double spiral_r, spiral_t;
  double last_x, first_x;
  double last_push_x;

  ros::Time started = ros::Time::now();
  updated_from_viz_ = false;

  while (isActive() && !updated_from_viz_)
  {
    // Pet the plug detector, to keep it going, because we need poses from
    // it
    node_.publish(TRACKER_ACTIVATE, empty);

    if (isPreemptRequested()) {
      ROS_ERROR("Deactivating because of preemption; plug pose never received.");
      deactivate(robot_actions::PREEMPTED, feedback);
      return waitForDeactivation(feedback);
    }
    ROS_INFO("Waiting on first plug pose");
    ros::Duration(0.5).sleep();
  }

  ros::Duration d(0.01);
  while (isActive())
  {
    if(ros::Time::now() - started > ros::Duration(99120.0)) {
      ROS_ERROR("Aborted: timeout");
      deactivate(robot_actions::ABORTED,feedback);
      break;
    }
    if (isPreemptRequested()) {
      ROS_ERROR("Deactivating because of preemption");
      deactivate(robot_actions::PREEMPTED, feedback);
      break;
    }

    node_.publish(TRACKER_ACTIVATE, empty);

    switch (state)
    {
    case APPROACHING: {

      tf::Transform error;
      tf::Transform mech_pose_desi;

      {
        boost::mutex::scoped_lock lock(from_viz_lock_);
        ros::Duration data_age = ros::Time::now() - viz_offset_from_viz_.stamp_;
        if (updated_from_viz_)
        {
          outlet_pose = viz_offset_from_viz_.inverse() * mech_offset_from_viz_;

          // Deals with the per-outlet offsets
          {
            double offset = getOffset(outlet_id.data);
            tf::Stamped<tf::Transform> high_def_in_outlet;
            TF_->lookupTransform("outlet_pose", "high_def_frame", ros::Time(0), high_def_in_outlet);
            tf::Vector3 v = -high_def_in_outlet.getOrigin();
            v *= (1 - offset);

            outlet_pose.getOrigin() -= v;
          }

          ROS_INFO("viz_offset_from_viz_ = (%.3lf, %.3lf, %.3lf)",
                   viz_offset_from_viz_.getOrigin()[0],
                   viz_offset_from_viz_.getOrigin()[1],
                   viz_offset_from_viz_.getOrigin()[2]);
          ROS_INFO("outlet_pose = (%.3lf, %.3lf, %.3lf)",
                   outlet_pose.getOrigin()[0],
                   outlet_pose.getOrigin()[1],
                   outlet_pose.getOrigin()[2]);

          tf::Pose offset_desi(tf::Quaternion(0,0,0), tf::Vector3(-MIN_STANDOFF, 0, 0));
          mech_pose_desi = offset_desi * outlet_pose;
          error = mech_offset_from_viz_ * mech_pose_desi.inverse();
          //error = viz_offset_from_viz_ * offset_desi.inverse();
          //mech_pose_desi = error.inverse() * mech_offset_from_viz_;
          updated_from_viz_ = false;
        }
        else if (data_age.toSec() > 10.0)
        {
          ROS_ERROR("Aborting: haven't gotten a plug pose in %.3lf seconds", data_age.toSec());
          deactivate(robot_actions::ABORTED, feedback);
          break;
        }
      }

      // Are we done approaching?
      if (ros::Time::now() - started > ros::Duration(10.0))
      {
        ROS_WARN("Approach took more than 10 seconds.  Moving on anyways, even though error is %.3lf", error.getOrigin().length());
        state = FIRST_TOUCH;
      }
      else if (error.getOrigin().length() <= 0.002)
      {
        ROS_INFO("Moving onto touching with an error of %.4lf", error.getOrigin().length());
        state = FIRST_TOUCH;
      }
      else
      {
        tf::Pose target = mech_pose_desi;
        const double MAX = 0.02;
        tf::Vector3 v = target.getOrigin() - mech_offset_from_viz_.getOrigin();
        if (v.length() > MAX) {
          target.setOrigin(mech_offset_from_viz_.getOrigin() + MAX * v / v.length());
        }


        // Move
        manipulation_msgs::TaskFrameFormalism tff_msg;
        tff_msg.header.frame_id = COMMAND_FRAME;
        tff_msg.header.stamp = viz_offset_from_viz_.stamp_;
        tff_msg.mode.vel.x = 3;
        tff_msg.mode.vel.y = 3;
        tff_msg.mode.vel.z = 3;
        tff_msg.mode.rot.x = 3;
        tff_msg.mode.rot.y = 3;
        tff_msg.mode.rot.z = 3;
        PoseTFToMsg(target, tff_msg.value);
        node_.publish(arm_controller_ + "/command", tff_msg);
        ros::Duration(1.0).sleep(); // Settle
      }

      break;
    }

    case FIRST_TOUCH: {

      const double WAIT = 1.0; // seconds
      const double SPEED = 0.01;

      tf::Pose desi = outlet_pose;
      desi.getOrigin() += tf::Vector3(0, -0.01, 0);
      manipulation_msgs::TaskFrameFormalism tff_msg;
      tff_msg.header.frame_id = COMMAND_FRAME;
      tff_msg.header.stamp = ros::Time::now() - ros::Duration(0.1);
      tff_msg.mode.vel.x = 2;
      tff_msg.mode.vel.y = 3;
      tff_msg.mode.vel.z = 3;
      tff_msg.mode.rot.x = 3;
      tff_msg.mode.rot.y = 3;
      tff_msg.mode.rot.z = 3;
      PoseTFToMsg(desi, tff_msg.value);
      tff_msg.value.vel.x = SPEED;
      node_.publish(arm_controller_ + "/command", tff_msg);

      ros::Time touching_started = ros::Time::now();
      double last_x = -99999999999.0;
      while (true)
      {
        {
          ros::Duration(WAIT).sleep();
          boost::mutex::scoped_lock(from_c_lock_);
          const double diff = 0.2 * (WAIT * SPEED);
          if (pose_from_mech_.getOrigin().x() <= last_x + diff)
          {
            ros::Duration(1.0).sleep();
            ROS_INFO("Finished FIRST_TOUCH because %.4lf is not further than %.4lf by %.4lf",
                     pose_from_mech_.getOrigin().x(), last_x, diff);
            last_x = std::max(pose_from_mech_.getOrigin().x(), last_x) + 0.001;
            break;
          }
          else if (ros::Time::now() - touching_started > ros::Duration(60.0))
          {
            ROS_ERROR("Aborting: Never seemed to touch the outlet");
            deactivate(robot_actions::ABORTED, empty_);
            break;
          }
          last_x = pose_from_mech_.getOrigin().x();
        }
      }
      first_x = last_x;

      if (!isActive())
        break;

      ros::Duration(1.0).sleep(); // Settle

      spiral_r = 0.0001;
      spiral_t = 0.0;
      //last_push_x = 999999;
      last_push_x = last_x;
      node_.setParam("/unplug/x_threshold", last_x - 0.02);
      state = SPIRALING;
      break;
    }

    case SPIRALING: {
      tf::Pose desi(outlet_pose);
      desi.getOrigin() += tf::Vector3(-MIN_STANDOFF-0.015, spiral_r * cos(spiral_t), spiral_r * sin(spiral_t));

      ROS_INFO("r = %.4lf,  t = %.3lf", spiral_r, spiral_t);
      ROS_INFO("desi = (%.3lf, %.3lf, %.3lf)",
               desi.getOrigin()[0],
               desi.getOrigin()[1],
               desi.getOrigin()[2]);


      // Move
      //
      double insertion_speed = 0.03;

      manipulation_msgs::TaskFrameFormalism tff_msg;
      tff_msg.header.frame_id = COMMAND_FRAME;
      tff_msg.header.stamp = ros::Time::now() - ros::Duration(0.1);
      tff_msg.mode.vel.x = 3;
      tff_msg.mode.vel.y = 3;
      tff_msg.mode.vel.z = 3;
      tff_msg.mode.rot.x = 3;
      tff_msg.mode.rot.y = 3;
      tff_msg.mode.rot.z = 3;
      PoseTFToMsg(desi, tff_msg.value);
      tff_msg.value.vel.x = first_x - 0.03;
      //tff_msg.value.vel.x = removal_speed;
      node_.publish(arm_controller_ + "/command", tff_msg);
      ros::Duration(0.2).sleep(); // Settle

      // Insert (push)
      tff_msg.header.stamp = ros::Time::now() - ros::Duration(0.1);
      //tff_msg.value.vel.x = outlet_pose.getOrigin().x();
      tff_msg.mode.vel.x = 2;
      tff_msg.value.vel.x = insertion_speed;

      //tff_msg.mode.vel.x = 2;
      //tff_msg.value.vel.x = 0.1;

      node_.publish(arm_controller_ + "/command", tff_msg);
      ros::Duration(2.0).sleep(); // Push in

      // Check
      {

        boost::mutex::scoped_lock lock(from_c_lock_);
#if 0
        ROS_INFO("Checking for insertion: %.3lf, %.3lf", pose_from_mech_.getOrigin().x(), outlet_pose.getOrigin().x());
        if (pose_from_mech_.getOrigin().x() > outlet_pose.getOrigin().x() + READY_TO_PUSH)
        {
          ROS_INFO("Inserted: %.3lf, %.3lf", pose_from_mech_.getOrigin().x(), outlet_pose.getOrigin().x());
          state = FORCING;
        }
#else
        ROS_INFO("Checking for successful insertion: %.3lf, %.3lf", pose_from_mech_.getOrigin().x(), last_push_x);
        if (pose_from_mech_.getOrigin().x() > first_x + 0.005)
        {
          state = FORCING;
        }
        if (pose_from_mech_.getOrigin().x() < first_x - 0.003)
        {
          // Did we just get out of the socket?
          ROS_INFO("I think we just got out of the socket");
          spiral_r = 0.0001;
          spiral_t = 0.0;
          first_x = pose_from_mech_.getOrigin().x();
          node_.setParam("/unplug/x_threshold", first_x - 0.02);
        }
        last_push_x = pose_from_mech_.getOrigin().x();
#endif
      }

      // Next step on the spiral
      spiral_t += std::min(SPIRAL_STEP / spiral_r, M_PI/2.0);
      spiral_r = 0.004 * spiral_t / (2 * M_PI);

      break;
    }

    case FORCING: {
      ROS_INFO("Forcing into the socket");
      ros::Duration(0.2).sleep();

      ros::Time started_forcing = ros::Time::now();

      tf::Pose orig_mech_pose;
      {
        boost::mutex::scoped_lock lock(from_c_lock_);
        orig_mech_pose = pose_from_mech_;
      }

      tff_msg_.mode.vel.x = 2;
      tff_msg_.mode.vel.y = 3;
      tff_msg_.mode.vel.z = 3;
      tff_msg_.mode.rot.x = 3;
      tff_msg_.mode.rot.y = 3;
      tff_msg_.mode.rot.z = 3;
      tff_msg_.value.vel.x = .1;
      tff_msg_.value.vel.y = orig_mech_pose.getOrigin().y();
      tff_msg_.value.vel.z = orig_mech_pose.getOrigin().z();
      orig_mech_pose.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);

      node_.publish(arm_controller_ + "/command", tff_msg_);

      double base_roll = tff_msg_.value.rot.x;
      double base_pitch = tff_msg_.value.rot.y;
      double base_yaw = tff_msg_.value.rot.z;
      while (ros::Time::now() - started_forcing < ros::Duration(15.0))
      {
	double time = ros::Time::now().toSec();
	tff_msg_.value.rot.x = base_roll  + 0.06 * sin(time*37.0*M_PI);
	tff_msg_.value.rot.y = base_pitch + 0.15 * sin(time*2.0 *M_PI);
	tff_msg_.value.rot.z = base_yaw   + 0.03 * sin(time*55.0 *M_PI);    
        node_.publish(arm_controller_ + "/command", tff_msg_);
        ros::Duration(0.001).sleep();
      }


      if (true)
      {
        boost::mutex::scoped_lock(from_c_lock_);
        ROS_INFO("Forcing successful: %.3lf, %.3lf", pose_from_mech_.getOrigin().x(), last_push_x);
	ros::Duration(10.0).sleep();
        deactivate(robot_actions::SUCCESS, empty_);
      }
      else
      {
        ROS_ERROR("Forcing failed to get us in");
        deactivate(robot_actions::ABORTED, empty_);
      }

      break;
    }
    }

    d.sleep();
  }

  return waitForDeactivation(feedback);
}

void PlugInAction::reset()
{
  last_standoff_ = 1.0e10;
  g_state_ = MEASURING;
  g_started_inserting_ = ros::Time::now();
  g_started_forcing_ = ros::Time::now();
  g_stopped_forcing_ = ros::Time::now();
}

void PlugInAction::plugMeasurementCB(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr &msg)
{
  plugs_core::PlugInState state_msg;

  //ROS_INFO("recieved plug_pose Msg in callback");

  if (!isActive())
    return;


  // Both are transforms from the outlet to the estimated plug pose
  robot_msgs::PoseStamped viz_offset_msg;
  tf::Stamped<tf::Transform> mech_offset;
  try {
    TF_->canTransform(COMMAND_FRAME, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
    TF_->transformPose(COMMAND_FRAME, *(msg.get()), viz_offset_msg);
    TF_->lookupTransform(COMMAND_FRAME, arm_controller_ + "/tool_frame", msg->header.stamp, mech_offset);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s: Error, transform exception: %s", action_name_.c_str(), ex.what());
    return;
  }

  // Commit
  if (from_viz_lock_.try_lock())
  {
    //boost::mutex::scoped_lock(from_viz_lock_);
    tf::PoseStampedMsgToTF(viz_offset_msg, viz_offset_from_viz_);
    mech_offset_from_viz_ = mech_offset;
    updated_from_viz_ = true;
    from_viz_lock_.unlock();
  }

}


void PlugInAction::controllerStateCB()
{
  if (!isActive())
    return;

  if (from_c_lock_.try_lock())
  {
    PoseMsgToTF(c_state_msg_.last_pose_meas, pose_from_mech_);
    from_c_lock_.unlock();
  }
}



/*
#if 0

  tff_msg_.header.stamp = msg->header.stamp;

  tf::Pose viz_offset;
  tf::PoseMsgToTF(viz_offset_msg.pose, viz_offset);
  PoseTFToMsg(viz_offset, state_msg.viz_offset);
  double standoff = std::max(MIN_STANDOFF, viz_offset.getOrigin().length()  * 2.5/4.0);

  // Computes the offset for movement
  tf::Pose viz_offset_desi;
  viz_offset_desi.setIdentity();
  viz_offset_desi.getOrigin().setX(-standoff);
  viz_offset_desi.getOrigin().setY(0.0);
  viz_offset_desi.getOrigin().setZ(0.0);
  PoseTFToMsg(viz_offset.inverse() * viz_offset_desi, state_msg.viz_error);
  mech_offset_desi_ = viz_offset.inverse() * viz_offset_desi * mech_offset_;

  prev_state_ = g_state_;

  switch (g_state_) {
    case MEASURING:
    {
x#if 0
      if (viz_offset.getOrigin().length() > 0.5 ||
          viz_offset.getRotation().getAngle() > (M_PI/6.0))
      {
        double ypr[3];
        btMatrix3x3(viz_offset.getRotation()).getEulerYPR(ypr[2], ypr[1], ypr[0]);
        ROS_ERROR("%s: Error, Crazy vision offset (%lf, (%lf, %lf, %lf))!!!.",
                  action_name_.c_str(), viz_offset.getOrigin().length(), ypr[0], ypr[1], ypr[2]);
        g_state_ = MEASURING;
        break;
      }
x#endif

      tf::Transform diff = viz_offset * prev_viz_offset_.inverse();
      if (diff.getOrigin().length() > 0.002 ||
          diff.getRotation().getAngle() > 0.035)
      {
        ROS_WARN("Vision estimate of the plug wasn't stable: %lf, %lf",
                 diff.getOrigin().length(), diff.getRotation().getAngle());
        g_state_ = MEASURING;
        break;
      }

      double error = sqrt(pow(viz_offset.getOrigin().y() - viz_offset_desi.getOrigin().y(), 2) +
                          pow(viz_offset.getOrigin().z() - viz_offset_desi.getOrigin().z(), 2));
      ROS_DEBUG("%s: Error = %0.6lf.", action_name_.c_str(), error);
      //if (error < 0.002 && last_standoff_ < MIN_STANDOFF + 0.002)
      if (error < 0.002 && viz_offset.getOrigin().x() > READY_TO_INSERT)
        g_state_ = INSERTING;
      else
        g_state_ = MOVING;

      break;
    }

    case MOVING: {
      g_state_ = MEASURING;
      break;
    }

    case INSERTING:
    {
      tf::Vector3 offset = viz_offset.getOrigin() - viz_offset_desi.getOrigin();
      ROS_DEBUG("%s: Offset: (% 0.3lf, % 0.3lf, % 0.3lf)", action_name_.c_str(), offset.x(), offset.y(), offset.z());
      if (g_started_inserting_ + ros::Duration(10.0) < ros::Time::now())
      {
        //if (offset.x() > SUCCESS_THRESHOLD) // if (in_outlet)
        if (viz_offset.getOrigin().x() > READY_TO_PUSH)  // if (in_outlet)
        {
          g_state_ = FORCING;
        }
        else
        {
          g_state_ = MEASURING;
        }
      }
      break;
    }
    case FORCING:
    {
      if (ros::Time::now() > g_started_forcing_ + ros::Duration(5.0))
      {
        if (viz_offset.getOrigin().x() > FORCING_SUCCESSFUL)
          g_state_ = HOLDING;
        else {
          ROS_ERROR("Forcing occurred, but the plug wasn't in");
          g_state_ = MEASURING;
        }
      }
      break;
    }
    case HOLDING:
    {
      break;
    }
  }

  state_msg.state = prev_state_;
  state_msg.next_state = g_state_;

  if (g_state_ != prev_state_)
  {
    switch (g_state_) {
      case MEASURING:
      {
        ROS_DEBUG("MEASURING");
        if (prev_state_ == INSERTING || prev_state_ == FORCING)
        {
          measure();
        }
        break;
      }
      case MOVING:
      {
        ROS_DEBUG("MOVING");
        move();
        break;
      }
      case INSERTING:
      {
        ROS_DEBUG("INSERTING");
        insert();
        break;
      }
      case FORCING:
      {
        ROS_DEBUG("FORCING");
        force();
        break;
      }
      case HOLDING:
      {
        ROS_DEBUG("HOLDING");
        hold();
        deactivate(robot_actions::SUCCESS, empty_);
        break;
      }
    }
  }


  last_standoff_ = standoff;
  prev_viz_offset_ = viz_offset;

  node_.publish(action_name_ + "/state", state_msg);

  return;

}
#endif
*/

/*
void PlugInAction::measure()
{
  if (!isActive())
    return;

  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = mech_offset_.getOrigin().x() - 0.05;  // backs off 5cm
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);
  g_stopped_forcing_ = ros::Time::now();
  last_standoff_ = 0.05;
  return;
}

void PlugInAction::move()
{
  if (!isActive())
    return;

  tf::Pose target = mech_offset_desi_;
  const double MAX = 0.02;
  tf::Vector3 v = target.getOrigin() - mech_offset_.getOrigin();
  if (v.length() > MAX) {
    target.setOrigin(mech_offset_.getOrigin() + MAX * v / v.length());
  }

  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = target.getOrigin().x();
  tff_msg_.value.vel.y = target.getOrigin().y();
  tff_msg_.value.vel.z = target.getOrigin().z();
  target.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  ROS_DEBUG("plublishing command to hybrid controller to move");
  node_.publish(arm_controller_ + "/command", tff_msg_);
  return;

}

void PlugInAction::insert()
{
  if (!isActive())
    return;
  g_started_inserting_ = ros::Time::now();

  tff_msg_.mode.vel.x = 2;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 0.003;
  tff_msg_.value.vel.y = mech_offset_desi_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_desi_.getOrigin().z();
  mech_offset_desi_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);

  return;
}

void PlugInAction::force()
{
  if (!isActive())
    return;

  g_started_forcing_ = ros::Time::now();

  tff_msg_.mode.vel.x = 1;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 2;
  tff_msg_.mode.rot.y = 2;
  tff_msg_.mode.rot.z = 2;
  tff_msg_.value.vel.x = 50;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  tff_msg_.value.rot.x = 0.0;
  tff_msg_.value.rot.y = 0.0;
  tff_msg_.value.rot.z = 0.0;
  tff_msg_.mode.vel.y = 2;
  tff_msg_.mode.vel.z = 2;
  tff_msg_.value.vel.y = 0.0;
  tff_msg_.value.vel.z = 0.0;


  tff_msg_.mode.vel.x = 2;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 0.1;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_desi_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);

  node_.publish(arm_controller_ + "/command", tff_msg_);

  double base_roll = tff_msg_.value.rot.x;
  double base_pitch = tff_msg_.value.rot.y;
  double base_yaw = tff_msg_.value.rot.z;
  while (ros::Time::now() - g_started_forcing_ < ros::Duration(5.0))
  {
    double time = ros::Time::now().toSec();
    tff_msg_.value.rot.x = base_roll  + 0.03 * sin(time*20.0*M_PI);
    tff_msg_.value.rot.y = base_pitch + 0.03 * sin(time*2.1*M_PI);
    tff_msg_.value.rot.z = base_yaw   + 0.01 * sin(time*5.3*M_PI);    
    node_.publish(arm_controller_ + "/command", tff_msg_);
    usleep(10000);
  }

  return;
}

void PlugInAction::hold()
{
  if (!isActive())
    return;

  tff_msg_.mode.vel.x = 1;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 4;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);
  return;
}
  */


}
