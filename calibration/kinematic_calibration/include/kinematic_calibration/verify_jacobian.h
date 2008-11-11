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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_VERIFY_JACOBIAN_H_
#define KINEMATIC_CALIBRATION_VERIFY_JACOBIAN_H_

#include <vector>
#include <string>
#include <fstream>

#include "kdl/chain.hpp"

#include "kinematic_calibration/link_param_jacobian_solver.h"


namespace kinematic_calibration
{

class ModelGetter
{
public:
  ModelGetter() { }
  ~ModelGetter() { }
  
  int OpenFile(const std::string& filename) ;
  
  KDL::Chain GetModel() ;
  
  void CloseFile() ;
private:
  std::ifstream infile_ ;
  
} ;

class JointStatesGetter
{
public:
  JointStatesGetter() { }
  ~JointStatesGetter() { }
  
  int OpenFile(const std::string& filename) ;
  int GetNextJointArray(KDL::JntArray& joint_array) ;
  void CloseFile() ;  
private:
  std::ifstream infile_ ;  
} ;

class JacobiansGetter
{
public:
  JacobiansGetter() { }
  ~JacobiansGetter() { }
  
  int OpenFile(const std::string& filename) ;
  int GetNextJacobian(LinkParamJacobian& jac ) ;
  void CloseFile() ;  
private:
  std::ifstream infile_ ;  
} ;




class VerifyJacobian
{
public:
  VerifyJacobian() ;
  ~VerifyJacobian() ;
  int ComputeMaxError(const std::string& model_file, const std::string& joint_params_file, const std::string& jacobians_file, double& max_error) ;
private:
  ModelGetter model_getter_ ;
  JointStatesGetter joint_params_getter_ ;
  JacobiansGetter jacobians_getter_ ;
} ;

}

#endif /* KINEMATIC_CALIBRATION_VERIFY_JACOBIAN_H_ */
