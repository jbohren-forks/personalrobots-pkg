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

#include <stdio.h>

#include "ros/node.h"
#include "robot_msgs/MechanismState.h"
#include "phase_space/PhaseSpaceSnapshot.h"
#include "robot_kinematics/robot_kinematics.h"

#include "kdl/chain.hpp"

#include <unistd.h>
#include <termios.h> 

using namespace std ;

namespace kinematic_calibration
{
  

class ArmPhaseSpaceGrabber : public ros::node
{

public:
  

  
  ArmPhaseSpaceGrabber() : ros::node("arm_phase_space_grabber")
  {
    marker_id_ = 1 ;
    
    subscribe("phase_space_snapshot", snapshot_, &ArmPhaseSpaceGrabber::SnapshotCallback, 2) ;
    subscribe("mechanism_state", mech_state_, &ArmPhaseSpaceGrabber::MechStateCallback, 2) ;
  }
  
  ~ArmPhaseSpaceGrabber()
  {
    unsubscribe("phase_space_snapshot") ;
    unsubscribe("MechanismState") ;
  }
  
  bool spin()
  {
    // Setup terminal settings for getchar
    const int fd = fileno(stdin);
    termios prev_flags ;
    tcgetattr(fd, &prev_flags) ;
    termios flags ;
    tcgetattr(fd,&flags);
    flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;     // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;     // block if waiting for char
    tcsetattr(fd,TCSANOW,&flags);
    
    KDL::Chain chain ;
    
    vector<string> joint_names ;
    
    bool result ;
    result = LoadJointNames("./joint_names.xml", joint_names) ;
    
    
    const string state_data_filename = "./state_data.txt" ;
    FILE* state_data_out ;
    state_data_out = fopen(state_data_filename.c_str(), "w") ;
    if (!state_data_out)
    {
      printf("Error opening state_data file\n") ;
      return false ;
    }
    fclose(state_data_out) ;
    
    while (ok())
    {
      char c = getchar() ;
      
      switch (c)
      {
        case ' ':
        {
          printf("Capturing...\n") ;
          phase_space::PhaseSpaceMarker cur_marker ;
          
          GetMarker(cur_marker, marker_id_) ;

          vector<double> joint_angles ;
          GetJointAngles(joint_names, joint_angles) ;
          
          // Print Marker Location
          printf("% 15.10f  % 15.10f  % 15.10f  ", cur_marker.location.x, cur_marker.location.y, cur_marker.location.z) ;
          
          // Print Joint Angles
          for (unsigned int i=0; i<joint_angles.size(); i++)
            printf("% 15.10f  ", joint_angles[i]) ;
          printf("\n") ;
          
          break ;
        } 
        case 'c':                       // Build kinematic chain for arm
        {
          robot_kinematics::RobotKinematics robot_kinematics ;
          string robot_desc ;
          param("robotdesc/pr2", robot_desc, string("")) ;
          printf("RobotDesc.length() = %u\n", robot_desc.length()) ;
          
          robot_kinematics.loadString(robot_desc.c_str()) ;

          robot_kinematics::SerialChain* serial_chain = robot_kinematics.getSerialChain("right_arm") ;
          
          if (serial_chain == NULL)
          {
            printf("Got NULL Chain\n") ;
            break ;
          }

          chain = serial_chain->chain ;
          printf("Extracted KDL Chain with %u Joints and %u segments\n", chain.getNrOfJoints(), chain.getNrOfSegments()) ;
          const string model_filename("./model.txt") ;
          printf("Writing chain to file: %s\n", model_filename.c_str()) ;
          FILE* model_out ;
          model_out = fopen(model_filename.c_str(), "w") ;

          if (!model_out)
          {
            printf("Error opening file\n") ;
            break ;
          }
          
          for (unsigned int i=0; i < chain.getNrOfSegments(); i++)
          {
            printf("Segment #%u\n", i) ;
            printf("   Translation: ") ;
            for (unsigned int j=0; j<3; j++)
              printf("% 15.10f  ", chain.getSegment(i).getFrameToTip().p(j) ) ;
            printf("\n") ;

            
            KDL::Vector rot_axis ;
            double rot_ang = chain.getSegment(i).getFrameToTip().M.GetRotAngle(rot_axis) ;

            printf("   Rotation:\n") ;
            printf("     Axis:      ") ;
            for (unsigned int j=0; j<3; j++)
              printf("% 15.10f  ", rot_axis(j)) ;
            printf("\n") ;
            printf("     Angle:     % 15.10f\n", rot_ang) ;

            KDL::Vector rot_vec = rot_ang * rot_axis ; 
            printf("     Product:   ") ;
            for (unsigned int j=0; j<3; j++)
              printf("% 15.10f  ", rot_vec(j)) ;
            printf("\n\n") ;
          }
          fclose(model_out) ;
          
          break ;
        } 
        default:
          break ;
      }
      
      usleep(1000) ;
      fflush(stdout) ;
    }
    printf("\n") ;
    tcsetattr(fd,TCSANOW, &prev_flags) ;         // Undo any terminal changes that we made
    return true ;
  }
  
  bool LoadJointNames(const string& joint_names_file, vector<string>& joint_names)
  {
    TiXmlDocument xml ;
    xml.LoadFile(joint_names_file.c_str()) ;
    TiXmlElement *config = xml.RootElement();
    
    if (config == NULL)
      return false ;
    
    // Extract the joint names
    TiXmlElement *joint_name_elem = config->FirstChildElement("joint_name") ;

    while (joint_name_elem)
    {

      const char* name = joint_name_elem->Attribute("name") ;
      printf("Found a joint_name: %s\n", name) ;
      joint_names.push_back(name) ;
      
      joint_name_elem = joint_name_elem->NextSiblingElement("joint_name") ;
    }
    return true ;
  }
  
  void GetMarker(phase_space::PhaseSpaceMarker& marker, int id)
  {
    bool marker_found = false ;
    
    // Grab phasespace marker
    while (!marker_found)
    {
      printf("  Looking for marker %u...", id) ;
      fflush(stdout) ;
      
      snapshot_lock_.lock() ;
      for (unsigned int i=0; i < safe_snapshot_.get_markers_size(); i++)
      {
        if (safe_snapshot_.markers[i].id == id)
        {
          marker_found = true ;
          marker = safe_snapshot_.markers[i] ;
          printf("Marker Found!\n") ;
          break ;
        }
      }
      snapshot_lock_.unlock() ;

      printf(".") ;
      fflush(stdout) ;
      usleep(100) ;
    }
  }
  
  void GetJointAngles(const vector<string>& names, vector<double>& angles)
  {
    // Grab mechanism state and put it in a local copy
    mech_lock_.lock() ;
    robot_msgs::MechanismState mech_state = safe_mech_state_ ;
    mech_lock_.unlock() ;    
    
    angles.resize(names.size()) ;
    
    for (unsigned int i=0; i < names.size(); i++)
    {
      for (unsigned int j=0; j < mech_state.get_joint_states_size(); j++)
      {
        if (names[i] == mech_state.joint_states[j].name)                         // See if we found the joint we're looking for
        {
          angles[i] = mech_state.joint_states[j].position ;
          break ;
        }
      }
    }
  }
  
  void MechStateCallback()
  {
    mech_lock_.lock() ;
    safe_mech_state_ = mech_state_ ;
    mech_lock_.unlock() ;
  }
  
  void SnapshotCallback()
  {
    snapshot_lock_.lock() ;
    safe_snapshot_ = snapshot_ ;
    snapshot_lock_.unlock() ;
  }
  
private:
  phase_space::PhaseSpaceSnapshot snapshot_ ;
  robot_msgs::MechanismState mech_state_ ;

  phase_space::PhaseSpaceSnapshot safe_snapshot_ ;
  int marker_id_ ;
  
  robot_msgs::MechanismState safe_mech_state_ ;
  
  ros::thread::mutex mech_lock_ ;
  ros::thread::mutex snapshot_lock_ ;
} ;

}

using namespace kinematic_calibration ;

int main(int argc, char** argv)
{
  ros::init(argc, argv) ;
  
  ArmPhaseSpaceGrabber grabber ; 
  grabber.spin() ;
  
  ros::fini() ;
  
  return 0 ;
}
