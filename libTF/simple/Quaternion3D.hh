// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
#ifndef QUATERNION3D_HH
#define QUATERNION3D_HH

#include <iostream>
#include <newmat/newmat.h>
#include <newmat/newmatio.h>
#include <math.h>

class Euler3D {
public:
  //Constructor
  Euler3D(double _x, double _y, double _z, double _yaw, double _pitch, double _roll);

  //Storage
  double  x,y,z,yaw,pitch,roll;

  

};

class Quaternion3D {
public:
  /** Constructors **/
  // Standard constructor which takes in 7 doubles
  Quaternion3D(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w);
  // Constructor from Matrix
  Quaternion3D(NEWMAT::Matrix matrixIn);
  
  /** Mutators **/
  // Set the values manually
  inline void Set(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w)
  {xt = _xt; yt = _yt; zt = _zt; xr = _xr; yr = _yr; zr = _zr; w = _w;} ;
  //Set the values from a matrix
  void fromMatrix(NEWMAT::Matrix matIn);
  void fromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll);
  void fromDH(double theta, double length, double distance, double alpha);

  
  /**** Utility Functions ****/
  static NEWMAT::Matrix matrixFromDH(double theta,
			      double length, double distance, double alpha);
  static NEWMAT::Matrix matrixFromEuler(double ax,
				 double ay, double az, double yaw,
				 double pitch, double roll);
  
  // Utility functions to normalize and get magnitude.
  void Normalize();
  double getMagnitude();

  /** Accessors **/
  // Return a Matrix
  NEWMAT::Matrix asMatrix();
  
  //Print as a matrix
  void printMatrix();


private:
  //Quaternion Storage
  double xt,yt,zt,xr,yr,zr,w;



};












#endif //QUATERNION3D_HH
