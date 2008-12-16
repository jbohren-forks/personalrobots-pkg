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

/**
 * \brief Used to extract a KDL model from a simple text file
 */
class ModelGetter
{
public:
  ModelGetter() { }
  ~ModelGetter() { }

  int openFile(const std::string& filename) ;

  KDL::Chain getModel() ;

  void closeFile() ;
private:
  std::ifstream infile_ ;

} ;

/**
 * \brief Used to incrementally extract KDL joint-array data from a text file.
 */
class JointStatesGetter
{
public:
  JointStatesGetter() { }
  ~JointStatesGetter() { }

  int openFile(const std::string& filename) ;
   //! brief Extracts the next joint array from the text file
  int getNextJointArray(KDL::JntArray& joint_array) ;
  void closeFile() ;
private:
  std::ifstream infile_ ;
} ;

class KDLVectorGetter
{
public:
  KDLVectorGetter() { }
  ~KDLVectorGetter() { }

  int openFile(const std::string& filename) ;
   //! brief Extracts the next KDL::Vector from the text file
  int getNextVec(KDL::Vector& vec) ;
  void closeFile() ;
private:
  std::ifstream infile_ ;
};

/**
 * \brief Used to incrementally extract LinkParamJacobians from a text file.
 */
class JacobiansGetter
{
public:
  JacobiansGetter() { }
  ~JacobiansGetter() { }

  int openFile(const std::string& filename) ;
  int getNextJacobian(LinkParamJacobian& jac ) ;
  void closeFile() ;
private:
  std::ifstream infile_ ;
} ;

}

#endif /* KINEMATIC_CALIBRATION_VERIFY_JACOBIAN_H_ */
