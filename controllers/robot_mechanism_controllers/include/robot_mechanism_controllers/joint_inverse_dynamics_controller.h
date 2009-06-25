/*
 * Copyright (c) 2009, Ruben Smits
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
 *     * Neither the name of Ruben Smits nor the names of its
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

/*
 * Author: Ruben Smits
 * based on cartesian_wrench_controller.h by Wim Meeussen for Willow Garage Inc.
 */

#ifndef ID_CONTROLLER_CONTROLLER_H
#define ID_CONTROLLER_CONTROLLER_H





#include <vector>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayacc.hpp>

#include <ros/node.h>
#include <std_msgs/Float64MultiArray.h>
#include <mechanism_model/controller.h>
#include <mechanism_model/chain.h>
#include <tf/transform_datatypes.h>
#include <diagnostic_msgs/DiagnosticMessage.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_chain.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>


namespace controller {

  class JointInverseDynamicsController : public Controller
  {
  public:
    JointInverseDynamicsController();
    ~JointInverseDynamicsController();

    bool initXml(mechanism::RobotState *robot_state, TiXmlElement *config);

    bool starting();
    void update();

    void command();

    // input of the controller
    KDL::JntArrayAcc jnt_posvelacc_desi_;
    KDL::Wrenches sgmnt_forces_;

  private:
    int counter;
    bool publishDiagnostics(int level, const std::string& message);

    double last_time_;
    ros::Node* node_;
    std::string controller_name_;
    mechanism::RobotState *robot_state_;
    mechanism::Chain chain_;
  
    //For the inverse dynamics:
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
    
    //For the acceleration controller
    KDL::JntArray jnt_acc_out_,jnt_tau_;
    KDL::JntArrayVel jnt_posvel_meas_;
    KDL::JntArrayAcc jnt_posvelacc_control_;
    double Kp_acc_,Kv_acc_,Kf_acc_,Kp_tau_,Kv_tau_;
    diagnostic_msgs::DiagnosticMessage diagnostics_;
    realtime_tools::RealtimePublisher <diagnostic_msgs::DiagnosticMessage> diagnostics_publisher_;
    ros::Time diagnostics_time_;
    ros::Duration diagnostics_interval_;

    std::vector<double> jnt_msg_std_,jnt_eff_std_;
    std_msgs::Float64MultiArray jnt_vel_msg_;
    std_msgs::Float64MultiArray jnt_msg_;

    //filters::FilterChain<double> *acc_vel_filter_,*vel_filter_,*acc_filter_;
    
  };

} // namespace


#endif
