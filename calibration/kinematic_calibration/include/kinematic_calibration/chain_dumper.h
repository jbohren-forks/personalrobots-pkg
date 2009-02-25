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

#ifndef KINEMATIC_CALIBRATION_CHAIN_DUMPER_H
#define KINEMATIC_CALIBRATION_CHAIN_DUMPER_H

#include <string>

#include "mechanism_model/chain.h"

namespace kinematic_calibration
{

class ChainDumper
{
public:
  static bool dumpChain(mechanism::Robot *robot, const std::string &root, const std::string &tip, const string& folder)
  {
    mechanism::Chain mech_chain ;
    bool success ;

    printf("dumpChain:: chain.init()\n") ;
    success = mech_chain.init(robot, root, tip) ;
    if (!success)
      return false ;
    printf("success!\n") ;

    KDL::Chain kdl_chain ;

    printf("dumpChain:: Converting to KDL\n") ;
    mech_chain.toKDL(kdl_chain) ;
    printf("success!\n") ;


    printf("Extracted KDL Chain with %u Joints and %u segments\n", kdl_chain.getNrOfJoints(), kdl_chain.getNrOfSegments()) ;
    const std::string model_filename = folder + "/model.txt" ;
    printf("Writing chain to file: %s\n", model_filename.c_str()) ;

    FILE* model_out ;
    model_out = fopen(model_filename.c_str(), "w") ;

    if (!model_out)
    {
      printf("Error opening file\n") ;
      return false ;
    }

    for (unsigned int i=0; i < kdl_chain.getNrOfSegments(); i++)
    {
      printf("Segment #%u\n", i) ;
      printf("   Translation: ") ;
      for (unsigned int j=0; j<3; j++)
        printf("% 15.10f  ", kdl_chain.getSegment(i).getFrameToTip().p(j) ) ;
      printf("\n") ;

      KDL::Vector rot_axis ;
      double rot_ang = kdl_chain.getSegment(i).getFrameToTip().M.GetRotAngle(rot_axis) ;

      printf("   Rotation:\n") ;
      printf("     Axis:      ") ;
      for (unsigned int j=0; j<3; j++)
        printf("% 15.10f  ", rot_axis(j)) ;
      printf("\n") ;
      printf("     Angle:     % 15.10f\n", rot_ang) ;

      printf("   JointType: ") ;
      switch (kdl_chain.getSegment(i).getJoint().getType())
      {
        case KDL::Joint::RotX : printf("RotX\n") ; break ;
        case KDL::Joint::RotY : printf("RotY\n") ; break ;
        case KDL::Joint::RotZ : printf("RotZ\n") ; break ;
        case KDL::Joint::TransX : printf("TransX\n") ; break ;
        case KDL::Joint::TransY : printf("TransY\n") ; break ;
        case KDL::Joint::TransZ : printf("TransZ\n") ; break ;
        case KDL::Joint::TransAxis : printf("TransAxis\n") ; break ;
        case KDL::Joint::RotAxis : printf("RotAxis\n") ; break ;
        case KDL::Joint::None : printf("Fixed\n") ; break ;
      }

      KDL::Vector rot_vec = rot_ang * rot_axis ;

      for (unsigned int j=0; j<3; j++)
        fprintf(model_out, "% 15.10f  ", kdl_chain.getSegment(i).getFrameToTip().p(j) ) ;
      for (unsigned int j=0; j<3; j++)
        fprintf(model_out, "% 15.10f  ", rot_vec(j)) ;
      fprintf(model_out, "\n") ;
      printf("\n\n") ;
    }
    fclose(model_out) ;

    FILE* joints_out ;
    const std::string joints_filename = folder + "/joints.txt" ;
    joints_out = fopen(joints_filename.c_str(), "w") ;
    if (!joints_out)
    {
      printf("Error opening file\n") ;
      return false ;
    }
    for (unsigned int i=0; i < kdl_chain.getNrOfSegments(); i++)
    {
      switch (kdl_chain.getSegment(i).getJoint().getType())
      {
        case KDL::Joint::RotX : fprintf(joints_out, "RotX\n") ; break ;
        case KDL::Joint::RotY : fprintf(joints_out, "RotY\n") ; break ;
        case KDL::Joint::RotZ : fprintf(joints_out, "RotZ\n") ; break ;
        case KDL::Joint::TransX : fprintf(joints_out, "TransX\n") ; break ;
        case KDL::Joint::TransY : fprintf(joints_out, "TransY\n") ; break ;
        case KDL::Joint::TransZ : fprintf(joints_out, "TransZ\n") ; break ;
        case KDL::Joint::None : fprintf(joints_out, "Fixed\n") ; break ;
      }
    }
    fclose(joints_out) ;


    return true ;
  }
} ;


}



#endif // KINEMATIC_CALIBRATION_CHAIN_DUMPER_H
