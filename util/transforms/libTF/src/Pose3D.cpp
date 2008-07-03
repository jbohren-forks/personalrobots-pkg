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
#include "libTF/Pose3D.h"

#include <cassert>

using namespace libTF;

Pose3D::Pose3D():
  xt(0), yt(0),zt(0),
  xr(1), yr(0), zr(0),w(0)
{
};


///Translation only constructor
Pose3D::Pose3D(double xt, double yt, double zt):
  xt(xt), yt(yt),zt(zt),
  xr(1), yr(0), zr(0),w(0)
{
}; 
/// Quaternion only constructor
Pose3D::Pose3D(double xr, double yr, double zr, double w):
  xt(0), yt(0),zt(0),
  xr(xr), yr(yr), zr(zr),w(w)
{
}; 
  /// Trans and Quat constructor
Pose3D::Pose3D(double xt, double yt, double zt, 
	       double xr, double yr, double zr, double w):
  xt(xt), yt(yt), zt(zt),
  xr(xr), yr(yr), zr(zr), w(w)
{
}; 


void Pose3D::setFromMatrix(const NEWMAT::Matrix& matIn)
{
  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html

  //  std::cout <<std::endl<<"Matrix in"<<std::endl<<matIn;

  const double * mat = matIn.Store();
  //Get the translations
  xt = mat[3];
  yt = mat[7];
  zt = mat[11];

  //TODO ASSERT others are zero and one as they should be


  double T  = 1 + mat[0] + mat[5] + mat[10];


  //  If the trace of the matrix is greater than zero, then
  //  perform an "instant" calculation.
  //	      Important note wrt. rouning errors:

  if ( T > 0.00000001 ) //to avoid large distortions!
    {
      double S = sqrt(T) * 2;
      xr = ( mat[9] - mat[6] ) / S;
      yr = ( mat[2] - mat[8] ) / S;
      zr = ( mat[4] - mat[1] ) / S;
      w = 0.25 * S;
    }
  //If the trace of the matrix is equal to zero then identify
  // which major diagonal element has the greatest value.
  //  Depending on this, calculate the following:
  else if ( mat[0] > mat[5] && mat[0] > mat[10] ) {// Column 0: 
        double S  = sqrt( 1.0 + mat[0] - mat[5] - mat[10] ) * 2;
        xr = 0.25 * S;
        yr = (mat[1] + mat[4] ) / S;
        zr = (mat[8] + mat[2] ) / S;
        w = (mat[6] - mat[9] ) / S;
      } else if ( mat[5] > mat[10] ) {// Column 1: 
        double S  = sqrt( 1.0 + mat[5] - mat[0] - mat[10] ) * 2;
        xr = (mat[1] + mat[4] ) / S;
        yr = 0.25 * S;
        zr = (mat[6] + mat[9] ) / S;
        w = (mat[8] - mat[2] ) / S;
      } else {// Column 2:
        double S  = sqrt( 1.0 + mat[10] - mat[0] - mat[5] ) * 2;
        xr = (mat[8] + mat[2] ) / S;
        yr = (mat[6] + mat[9] ) / S;
        zr = 0.25 * S;
        w = (mat[1] - mat[4] ) / S;
      }

  //  std::cout << "setFromMatrix" << xt  <<" "<< yt  <<" "<< zt  <<" "<< xr  <<" "<<yr <<" "<<zr <<" "<<w <<std::endl;
  //std::cout << "asMatrix" <<std::endl<< asMatrix();

};

void Pose3D::setFromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll)
{
  setFromMatrix(Pose3D::matrixFromEuler(_x,_y,_z,_yaw,_pitch,_roll));
};

void Pose3D::setFromEuler(Position &pos, Euler &euler)
{
    setFromEuler(pos.x, pos.y, pos.z, euler.yaw, euler.pitch, euler.roll);
}

void Pose3D::setFromDH(double length, double alpha, double offset, double theta)
{
  setFromMatrix(Pose3D::matrixFromDH(length, alpha, offset, theta));
};


NEWMAT::Matrix Pose3D::matrixFromEuler(double ax,
			       double ay, double az, double yaw,
			       double pitch, double roll)
{
  NEWMAT::Matrix matrix(4,4);
  double ca = cos(yaw);
  double sa = sin(yaw);
  double cb = cos(pitch);
  double sb = sin(pitch);
  double cg = cos(roll);
  double sg = sin(roll);
  double sbsg = sb*sg;
  double sbcg = sb*cg;


  double* matrix_pointer = matrix.Store();

  matrix_pointer[0] =  ca*cb;
  matrix_pointer[1] = (ca*sbsg)-(sa*cg);
  matrix_pointer[2] = (ca*sbcg)+(sa*sg);
  matrix_pointer[3] = ax;

  matrix_pointer[4] = sa*cb;
  matrix_pointer[5] = (sa*sbsg)+(ca*cg);
  matrix_pointer[6] = (sa*sbcg)-(ca*sg);
  matrix_pointer[7] = ay;

  matrix_pointer[8] = -sb;
  matrix_pointer[9] = cb*sg;
  matrix_pointer[10] = cb*cg;
  matrix_pointer[11] = az;

  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return matrix;
};


// Math from http://en.wikipedia.org/wiki/Robotics_conventions
NEWMAT::Matrix Pose3D::matrixFromDH(double length,
			    double alpha, double offset, double theta)
{
  NEWMAT::Matrix matrix(4,4);
  
  double ca = cos(alpha);
  double sa = sin(alpha);
  double ct = cos(theta);
  double st = sin(theta);
  
  double* matrix_pointer = matrix.Store();
  
  matrix_pointer[0] =  ct;
  matrix_pointer[1] = -st*ca;
  matrix_pointer[2] = st*sa;
  matrix_pointer[3] = length * ct;
  matrix_pointer[4] = st;
  matrix_pointer[5] = ct*ca;
  matrix_pointer[6] = -ct*sa;
  matrix_pointer[7] = length*st;
  matrix_pointer[8] = 0;
  matrix_pointer[9] = sa;
  matrix_pointer[10] = ca;
  matrix_pointer[11] = offset;
  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return matrix;
};


Pose3D& Pose3D::operator=(const Pose3D & input)
{
  xt = input.xt;
  yt = input.yt;
  zt = input.zt;
  xr = input.xr;
  yr = input.yr;
  zr = input.zr;
  w  = input.w ;

  return *this;
};


Pose3D::Euler Pose3D::eulerFromMatrix(const NEWMAT::Matrix & matrix_in, unsigned int solution_number)
{

  Euler euler_out;
  Euler euler_out2; //second solution
  //get the pointer to the raw data
  double* matrix_pointer = matrix_in.Store();

  // Check that pitch is not at a singularity
  if (fabs(matrix_pointer[8]) >= 1)
    {
      euler_out.pitch = - asin(matrix_pointer[8]);
      euler_out2.pitch = M_PI - euler_out.pitch;

      euler_out.roll = atan2(matrix_pointer[9]/cos(euler_out.pitch), 
			     matrix_pointer[10]/cos(euler_out.pitch));
      euler_out2.roll = atan2(matrix_pointer[9]/cos(euler_out2.pitch), 
			      matrix_pointer[10]/cos(euler_out2.pitch));
      
      euler_out.yaw = atan2(matrix_pointer[4]/cos(euler_out.pitch), 
			    matrix_pointer[1]/cos(euler_out.pitch));
      euler_out2.yaw = atan2(matrix_pointer[4]/cos(euler_out2.pitch), 
			     matrix_pointer[1]/cos(euler_out2.pitch));
    }
  else
    {
      euler_out.yaw = 0;
      euler_out2.yaw = 0;

      // From difference of angles formula
      double delta = atan2(matrix_pointer[1],matrix_pointer[2]);
      if (matrix_pointer[8] > 0)  //gimbal locked up
	{
	  euler_out.pitch = M_PI / 2.0;
	  euler_out2.pitch = M_PI / 2.0;
	  euler_out.roll = euler_out.pitch + delta;
	  euler_out2.roll = euler_out.pitch + delta;
	}
      else // gimbal locked down
	{
	  euler_out.pitch = -M_PI / 2.0;
	  euler_out2.pitch = -M_PI / 2.0;
	  euler_out.roll = -euler_out.pitch + delta;
	  euler_out2.roll = -euler_out.pitch + delta;
	}
    }
  
  if (solution_number == 1)
    return euler_out;
  else
    return euler_out2;
};

Pose3D::Position Pose3D::positionFromMatrix(const NEWMAT::Matrix & matrix_in)
{

  Position position;
  //get the pointer to the raw data
  double* matrix_pointer = matrix_in.Store();
  //Pass through translations
  position.x = matrix_pointer[3];
  position.y = matrix_pointer[7];
  position.z = matrix_pointer[11];
  return position;
};

void Pose3D::Normalize()
{
  double mag = getMagnitude();
  xr /= mag;
  yr /= mag;
  zr /= mag;
  w /= mag;
};

double Pose3D::getMagnitude()
{
  return sqrt(xr*xr + yr*yr + zr*zr + w*w);
};

NEWMAT::Matrix Pose3D::asMatrix()
{
  
  NEWMAT::Matrix outMat(4,4);
  
  double * mat = outMat.Store();

  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html
  double xx      = xr * xr;
  double xy      = xr * yr;
  double xz      = xr * zr;
  double xw      = xr * w;
  double yy      = yr * yr;
  double yz      = yr * zr;
  double yw      = yr * w;
  double zz      = zr * zr;
  double zw      = zr * w;
  mat[0]  = 1 - 2 * ( yy + zz );
  mat[1]  =     2 * ( xy - zw );
  mat[2]  =     2 * ( xz + yw );
  mat[4]  =     2 * ( xy + zw );
  mat[5]  = 1 - 2 * ( xx + zz );
  mat[6]  =     2 * ( yz - xw );
  mat[8]  =     2 * ( xz - yw );
  mat[9]  =     2 * ( yz + xw );
  mat[10] = 1 - 2 * ( xx + yy );
  mat[12]  = mat[13] = mat[14] = 0;
  mat[3] = xt;
  mat[7] = yt;
  mat[11] = zt;
  mat[15] = 1;
    

  return outMat;
};

NEWMAT::Matrix Pose3D::getInverseMatrix()
{
  return asMatrix().i();
};

Pose3D::Quaternion Pose3D::getQuaternion(void) const
{
  Quaternion quat;
  quat.x = xr;
  quat.y = yr;
  quat.z = zr;
  quat.w = w;
  return quat;
};

Pose3D::Position Pose3D::getPosition(void) const
{
  Position pos;
  pos.x = xt;
  pos.y = yt;
  pos.z = zt;
  return pos;
};

void Pose3D::setPosition(double x, double y, double z)
{
    xt = x;
    yt = y;
    zt = z;
}

void Pose3D::setPosition(Position &pos)
{
    xt = pos.x;
    yt = pos.y;
    zt = pos.z;
}

void Pose3D::setQuaternion(double x, double y, double z, double _w)
{
    xr = x;
    yr = y;
    zr = z;
    w  = _w;
}

void Pose3D::setQuaternion(Quaternion &quat)
{
    xr = quat.x;
    yr = quat.y;
    zr = quat.z;
    w  = quat.w;
}

