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

#include "kinematic_calibration/unittest_io.h"
#include "kinematic_calibration/link_param_jacobian_solver.h"

using namespace KDL ;
using namespace kinematic_calibration ;
using namespace std ;


// ********* MODEL GETTER *********

int ModelGetter::openFile(const string& filename)
{
  infile_.open(filename.c_str(), ios::in) ;

  if (infile_)
    return 0 ;

  return -1 ;
}

void ModelGetter::closeFile()
{
  infile_.close() ;
}

Chain ModelGetter::getModel()
{
  const int MAX_LEN = 1024 ;
  char cur_line[1024] ;

  Chain chain ;
  while (!infile_.eof())
  {
    double model[6] ;
    infile_.getline(cur_line, MAX_LEN) ;
    int num_read = sscanf(cur_line, "%lf %lf %lf %lf %lf %lf", &model[0], &model[1], &model[2], &model[3], &model[4], &model[5]) ;
    if (num_read < 6)
      break ;

    // Now add the data to the chain
    Joint J(Joint::RotZ) ;
    Vector rot_axis(model[3], model[4], model[5]) ;           // KDL doesn't need vector to be normalized. It even behaves nicely with vector=[0 0 0]
    double rot_ang = rot_axis.Norm() ;
    Rotation R(Rotation::Rot(rot_axis, rot_ang)) ;
    Vector trans(model[0], model[1], model[2]) ;
    chain.addSegment(Segment(J, Frame(R, trans))) ;
  }

  return chain ;
}

// ********* JOINT STATES GETTER *********

int JointStatesGetter::openFile(const string& filename)
{
  infile_.open(filename.c_str(), ios::in) ;

  if (infile_)
    return 0 ;

  return -1 ;
}

void JointStatesGetter::closeFile()
{
  infile_.close() ;
}

int JointStatesGetter::getNextJointArray(JntArray& joint_array)
{
  const unsigned int N = joint_array.rows() ;

  for (unsigned int i=0; i<N; i++)
    infile_ >> joint_array(i) ;

  if (infile_.eof())
    return -1 ;

  return 0 ;
}

// ********* KDLVectorGetter GETTER *********
int KDLVectorGetter::openFile(const string& filename)
{
  infile_.open(filename.c_str(), ios::in) ;

  if (infile_)
    return 0 ;

  return -1 ;
}

void KDLVectorGetter::closeFile()
{
  infile_.close() ;
}

int KDLVectorGetter::getNextVec(KDL::Vector& vec)
{

  double x, y, z ;

  infile_ >> x ;
  infile_ >> y ;
  infile_ >> z ;

  if (infile_.eof())
    return -1 ;

  vec = KDL::Vector(x,y,z) ;

  return 0 ;
}

// ********* JACOBIANS GETTER *********
int JacobiansGetter::openFile(const string& filename)
{
  infile_.open(filename.c_str(), ios::in) ;

  if (infile_)
    return 0 ;

  return -1 ;
}

void JacobiansGetter::closeFile()
{
  infile_.close() ;
}

int JacobiansGetter::getNextJacobian(LinkParamJacobian& jac )
{
  if (infile_.eof())
    return -1 ;

  const unsigned int N = jac.links_.size() ;

  for (unsigned int h=0; h<3; h++)                        // Iterate over x,y,z velocity output
  {
    for (unsigned int i=0; i<N; i++)                      // Iterate over each twist
    {
      for (unsigned int j=0; j<3; j++)                    // Iterate over xyz translational
        infile_ >> jac.links_[i].trans_[j].vel(h) ;
      for (unsigned int j=0; j<3; j++)                    // Iterate over xyz rotational movement
        infile_ >> jac.links_[i].rot_[j].vel(h) ;
    }
  }

  return 0 ;
}


