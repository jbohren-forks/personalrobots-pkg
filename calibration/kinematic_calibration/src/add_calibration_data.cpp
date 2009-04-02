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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Vijay Pradeep


#include <string>
#include <list>
#include <utility>

#include "ros/node.h"
#include "mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"
#include "hardware_interface/hardware_interface.h"

// Messages
#include "kinematic_calibration/LinkParamsDH.h"
#include "kinematic_calibration/RobotCal.h"

using namespace std ;
using namespace kinematic_calibration ;

int getCamCorrection(list< pair<string, double> >& calib_params, mechanism::Robot& robot, tf::Transform opt_final_T, const string& chain_type) ;
int getHeadInitialCorrection(list< pair<string, double> >& calib_params, mechanism::Robot& robot, tf::Transform opt_initial_T) ;
int getArmChainCorrections(list< pair<string, double> >& calib_params,
                           mechanism::Robot& robot, const vector<LinkParamsDH>& arm_dh) ;
int getHeadChainCorrections(list< pair<string, double> >& calib_params,
                            mechanism::Robot& robot, const vector<LinkParamsDH>& head_dh) ;
tf::Transform getHeadTiltToCam(mechanism::Robot& robot, const string& chain_type ) ;
string dumpToXML(const list< pair<string, double> >& calib_params) ;


class CalibrationAdder
{
public:
  CalibrationAdder(ros::Node* node) : node_(node), hw_(0)
  {
    printf("Getting robot description from param server...") ;
    fflush(stdout) ;
    string robot_desc ;
    bool success ;
    success = node_->getParam("robotdesc/pr2", robot_desc) ;
    if (!success)
    {
      printf("ERROR: Could not access robot_desc/pr2 from param server\n") ;
    }
    else
      printf("Success!\n") ;

    printf("Parsing robotdesc/pr2...") ;
    fflush(stdout) ;
    TiXmlDocument doc ;
    doc.Parse(robot_desc.c_str()) ;
    printf("Success!\n") ;

    TiXmlElement *root = doc.FirstChildElement("robot");
    if (!root)
    {
      printf("Error finding 'robot' tag in xml\n");
    }

    printf("Initializing Robot...") ;
    fflush(stdout) ;
    robot_.hw_ = &hw_ ;
    robot_.initXml(root) ;
    printf("Success!\n") ;

    printf("Displaying all the links:\n") ;
    for (unsigned int i=0; i<robot_.links_.size(); i++)
    {
      printf("%02u) %p ", i, robot_.links_[i]) ;
      fflush(stdout) ;
      printf("%s\n", robot_.links_[i]->name_.c_str())  ;
    }

    printf("Displaying all the joints:\n") ;
    for (unsigned int i=0; i<robot_.joints_.size(); i++)
    {
      printf("%02u) %p ", i, robot_.joints_[i]) ;
      fflush(stdout) ;
      printf("%s\n", robot_.joints_[i]->name_.c_str()) ;
    }


    node_->subscribe("calibrated_robot", robot_cal_, &CalibrationAdder::computeCalOffsets, this, 1) ;

  }

private:
  ros::Node* node_ ;
  HardwareInterface hw_ ;
  mechanism::Robot robot_ ;
  RobotCal robot_cal_ ;         // Receives robot calibration data over ros

  void computeCalOffsets()
  {
    list< pair<string, double> > calib_params ;

    tf::Transform stereo_final_T ;
    tf::TransformMsgToTF(robot_cal_.stereo_final, stereo_final_T) ;
    getCamCorrection(calib_params, robot_, stereo_final_T, "stereo") ;

    tf::Transform high_res_final_T ;
    tf::TransformMsgToTF(robot_cal_.high_res_final, high_res_final_T) ;
    getCamCorrection(calib_params, robot_, high_res_final_T, "high_res") ;

    tf::Transform head_initial_T ;
    tf::TransformMsgToTF(robot_cal_.head_initial, head_initial_T) ;
    getHeadInitialCorrection(calib_params, robot_, head_initial_T) ;


    getArmChainCorrections(calib_params, robot_, robot_cal_.arm_chain) ;
    getHeadChainCorrections(calib_params, robot_, robot_cal_.head_chain) ;


    //getStereocamCorrection(calib_params, robot, hires_final_params, "hires") ;

    string xml_data = dumpToXML(calib_params) ;
  }

} ;

tf::Transform paramsToTransform(double params[])
{
  tf::Vector3 trans_vec(params[0], params[1], params[2]) ;
  tf::Vector3 rot_vec(params[3], params[4], params[5]) ;
  double rot_angle = rot_vec.length() ;
  if (rot_angle < 1e-7)
    rot_vec = tf::Vector3(0,0,1) ;
  else
    rot_vec /= rot_angle ;

  tf::Transform T(tf::Quaternion(rot_vec, rot_angle), trans_vec) ;
  return T ;
}

void displayTransform(const tf::Transform T, string prefix="")
{
  const char* p = prefix.c_str() ;
  printf("%sRotation Matrix:\n", p) ;
  for (int i=0; i<3; i++)
  {
    printf("%s   % 5.3f  % 5.3f  % 5.3f\n", p,
           T.getBasis().getRow(i).x(),
           T.getBasis().getRow(i).y(),
           T.getBasis().getRow(i).z()) ;
  }

  printf("%sOrigin Vector:\n", p) ;
  printf("%s   % 5.3f  % 5.3f  % 5.3f\n", p,
           T.getOrigin().x(),
           T.getOrigin().y(),
           T.getOrigin().z()) ;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv) ;
  ros::Node node("add_cal_data") ;

  CalibrationAdder cal_adder(&node) ;

  node.spin() ;

//  string xml_data = dumpToXML(calib_params) ;
}

tf::Transform getHeadTiltToCam(mechanism::Robot& robot, const string& chain_type )
{
  mechanism::Link* mech_chain[3] ;

  if (chain_type == "stereo")
  {
    mech_chain[0] = robot.getLink("head_plate_frame") ;
    mech_chain[1] = robot.getLink("stereo_link") ;
    mech_chain[2] = robot.getLink("stereo_optical_frame") ;
  }
  else
  {
    mech_chain[0] = robot.getLink("head_plate_frame") ;
    mech_chain[1] = robot.getLink("high_def_frame") ;
    mech_chain[2] = robot.getLink("high_def_optical_frame") ;
  }

  // Error checking
  for (int i=0; i<3; i++)
  {
    if (!mech_chain[i])
    {
      printf("Error extracting link #%i of chain\n", i) ;
      return tf::Transform() ;
    }
    printf("%i) Found %s\n", i, mech_chain[i]->name_.c_str()) ;
  }

  // Extract Chain
  tf::Transform mech_chain_T ;
  mech_chain_T.setIdentity() ;
  for (int i=0; i<3; i++)
  {
    mech_chain_T = mech_chain_T * mech_chain[i]->getOffset() ;
    mech_chain_T = mech_chain_T * mech_chain[i]->getRotation() ;
  }
  return mech_chain_T ;
}

int getCamCorrection(list< pair<string, double> >& calib_params, mechanism::Robot& robot, tf::Transform opt_final_T, const string& chain_type)
{
  tf::Transform mech_chain_T = getHeadTiltToCam(robot, chain_type) ;

  double chain_yaw, chain_pitch, chain_roll ;
  mech_chain_T.getBasis().getEulerZYX(chain_yaw, chain_pitch, chain_roll) ;
  printf("Roll: %f   Pitch: %f   Yaw: %f\n", chain_roll, chain_pitch, chain_yaw) ;


  //printf("CameraChain Transform:\n") ;
  //displayTransform(mech_chain_T) ;
  //printf("\n") ;

  // The DH model's frame for the HeadTilt frame doesn't line up with the robot model's,
  //   so apply an initial transform to the DH model to make them match.
  tf::Transform opt_initial_T = tf::Transform(tf::Quaternion(-sqrt(2)/2, 0, 0, sqrt(2)/2),
                                                    tf::Vector3(0, 0, 0)) ;
  // The transform determined by the DH model
  //tf::Transform opt_final_T = paramsToTransform(params) ;

  // The transform determined by the DH model, along with the extra initial transform
  tf::Transform opt_total_T = opt_initial_T * opt_final_T ;

  //printf("StereoCam Total Transform:\n") ;
  //displayTransform(opt_total_T) ;
  //printf("\n") ;

  double opt_yaw, opt_pitch, opt_roll ;
  opt_total_T.getBasis().getEulerZYX(opt_yaw, opt_pitch, opt_roll) ;
  printf("Roll: %f   Pitch: %f   Yaw: %f\n", opt_roll, opt_pitch, opt_yaw) ;

  if (chain_type == "stereo")
  {
    calib_params.push_back( pair<string, double>("cal_stereo_x", opt_total_T.getOrigin().x() - mech_chain_T.getOrigin().x()) ) ;
    calib_params.push_back( pair<string, double>("cal_stereo_y", opt_total_T.getOrigin().y() - mech_chain_T.getOrigin().y()) ) ;
    calib_params.push_back( pair<string, double>("cal_stereo_z", opt_total_T.getOrigin().z() - mech_chain_T.getOrigin().z()) ) ;
    calib_params.push_back( pair<string, double>("cal_stereo_roll",  opt_roll  - chain_roll ) ) ;
    calib_params.push_back( pair<string, double>("cal_stereo_pitch", opt_pitch - chain_pitch) ) ;
    calib_params.push_back( pair<string, double>("cal_stereo_yaw",   opt_yaw   - chain_yaw  ) ) ;
  }
  else
  {
    calib_params.push_back( pair<string, double>("cal_high_def_x", opt_total_T.getOrigin().x() - mech_chain_T.getOrigin().x()) ) ;
    calib_params.push_back( pair<string, double>("cal_high_def_y", opt_total_T.getOrigin().y() - mech_chain_T.getOrigin().y()) ) ;
    calib_params.push_back( pair<string, double>("cal_high_def_z", opt_total_T.getOrigin().z() - mech_chain_T.getOrigin().z()) ) ;
    calib_params.push_back( pair<string, double>("cal_high_def_roll",  opt_roll  - chain_roll ) ) ;
    calib_params.push_back( pair<string, double>("cal_high_def_pitch", opt_pitch - chain_pitch) ) ;
    calib_params.push_back( pair<string, double>("cal_high_def_yaw",   opt_yaw   - chain_yaw  ) ) ;
  }

  return 0 ;
}

int getHeadInitialCorrection(list< pair<string, double> >& calib_params, mechanism::Robot& robot, tf::Transform opt_initial_T)
{
  mechanism::Link* mech_head ;
  mech_head = robot.getLink("head_pan_link") ;
  if (!mech_head)
  {
    printf("Error extracting head_pan_link\n") ;
    return -1 ;
  }
  printf("Found %s\n", mech_head->name_.c_str()) ;

  double chain_yaw, chain_pitch, chain_roll ;
  mech_head->getRotation().getBasis().getEulerZYX(chain_yaw, chain_pitch, chain_roll) ;

  //tf::Transform opt_initial_T = paramsToTransform(head_initial_params) ;

  double opt_yaw, opt_pitch, opt_roll ;
  opt_initial_T.getBasis().getEulerZYX(opt_yaw, opt_pitch, opt_roll) ;

  calib_params.push_back( pair<string, double>("cal_head_x", opt_initial_T.getOrigin().x() - mech_head->getOffset().getOrigin().x()) ) ;
  calib_params.push_back( pair<string, double>("cal_head_y", opt_initial_T.getOrigin().y() - mech_head->getOffset().getOrigin().y()) ) ;
  calib_params.push_back( pair<string, double>("cal_head_z", opt_initial_T.getOrigin().z() - mech_head->getOffset().getOrigin().z()) ) ;
  calib_params.push_back( pair<string, double>("cal_head_roll",  opt_roll  - chain_roll ) ) ;
  calib_params.push_back( pair<string, double>("cal_head_pitch", opt_pitch - chain_pitch) ) ;
  calib_params.push_back( pair<string, double>("cal_head_yaw",   opt_yaw   - chain_yaw  ) ) ;

  return 0 ;
}

int getArmChainCorrections(list< pair<string, double> >& calib_params,
                           mechanism::Robot& robot, const vector<LinkParamsDH>& arm_dh)
{
  assert(arm_dh.size() == 7) ;
  calib_params.push_back( pair<string, double>("cal_r_shoulder_pan_flag",   arm_dh[0].theta) ) ;
  calib_params.push_back( pair<string, double>("cal_r_shoulder_lift_flag",  arm_dh[1].theta - M_PI/2) ) ;
  calib_params.push_back( pair<string, double>("cal_r_upper_arm_roll_flag", arm_dh[2].theta ) ) ;
  calib_params.push_back( pair<string, double>("cal_r_elbow_flex_flag",     arm_dh[3].theta ) ) ;
  calib_params.push_back( pair<string, double>("cal_r_forearm_roll_flag",  -arm_dh[4].theta ) ) ;
  calib_params.push_back( pair<string, double>("cal_r_wrist_flex_flag",    -arm_dh[5].theta ) ) ;
  calib_params.push_back( pair<string, double>("cal_r_wrist_roll_flag",     arm_dh[6].theta ) ) ;

  calib_params.push_back( pair<string, double>("cal_r_shoulder_pan_gearing",   arm_dh[0].gearing) ) ;
  calib_params.push_back( pair<string, double>("cal_r_shoulder_lift_gearing",  arm_dh[1].gearing) ) ;
  calib_params.push_back( pair<string, double>("cal_r_upper_arm_roll_gearing", arm_dh[2].gearing) ) ;

  return 0 ;
}

int getHeadChainCorrections(list< pair<string, double> >& calib_params,
                            mechanism::Robot& robot, const vector<LinkParamsDH>& head_dh)
{
  assert(head_dh.size() == 2) ;
  calib_params.push_back( pair<string, double>("cal_head_pan_flag",  head_dh[0].theta) ) ;
  calib_params.push_back( pair<string, double>("cal_head_tilt_flag", head_dh[1].theta) ) ;

  calib_params.push_back( pair<string, double>("cal_head_pan_gearing",  head_dh[0].gearing) ) ;
  calib_params.push_back( pair<string, double>("cal_head_tilt_gearing", head_dh[1].gearing) ) ;

  return 0 ;
}

string dumpToXML(const list< pair<string, double> >& calib_params)
{
  list< pair<string, double> >::const_iterator it ;

  for (it = calib_params.begin(); it != calib_params.end(); ++it )
  {
    printf("<property name=\"%s\" value=\"%f\" />\n", it->first.c_str(), it->second) ;
  }

  return "" ;
}






