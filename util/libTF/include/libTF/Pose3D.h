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

#ifndef LIBTF_POSE3D_HH
#define LIBTF_POSE3D_HH

#include <iostream>
#include <vector>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <pthread.h>
#include <cmath>

#include <std_msgs/Pose3D.h>

namespace libTF
{
/** \brief A struct to represent the translational component (a point) */
struct Position
{
  double x,y,z;
  /** \brief Constructor */
  Position():x(0),y(0),z(0){;};
  /** \brief Constructor */
  Position(double x, double y, double z):x(x),y(y),z(z){;};
  /** \brief operator overloading for the + operator */
  Position  operator+(const Position &rhs){
    Position result;
    result.x = x + rhs.x;
    result.y = y + rhs.y;
    result.z = z + rhs.z;
    return result;
  }

  /** \brief operator overloading for the += operator */
  Position & operator+=(const Position &rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  /** \brief operator overloading for the - operator */
  Position  operator-(const Position &rhs){
    Position result;
    result.x = x - rhs.x;
    result.y = y - rhs.y;
    result.z = z - rhs.z;
    return result;
  }

  /** \brief operator overloading for the -= operator */
  Position & operator-=(const Position &rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  /** \brief operator overloading for the *= operator */
  Position & operator*=(double rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }

  /** \brief operator overloading for the * operator */
  Position  operator*(double rhs){
    Position result;
    result.x = x*rhs;
    result.y = y*rhs;
    result.z = z*rhs;
    return result;
  }

  /** \brief Rotate a position about the z-axis */
  Position rot2D(double angle){
    Position result;
    double cosa = cos(angle);
    double sina = sin(angle);
    result.x = cosa*x - sina*y;
    result.y = sina*x + cosa*y;
    result.z = z;
    return result;



  }
};

/** \brief A struct to represent vectors */
struct Vector
{
  double x,y,z;

  /** \brief Constructor */
  Vector():x(0),y(0),z(0){;};
  /** \brief Constructor */
  Vector(double x, double y, double z):x(x),y(y),z(z){;};

  /** \brief operator overloading for the + operator */
  Vector  operator+(const Vector &rhs){
    Vector result;
    result.x = x + rhs.x;
    result.y = y + rhs.y;
    result.z = z + rhs.z;
    return result;
  }

  /** \brief operator overloading for the += operator */
  Vector & operator+=(const Vector &rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  /** \brief operator overloading for the - operator */
  Vector  operator-(const Vector &rhs){
    Vector result;
    result.x = x - rhs.x;
    result.y = y - rhs.y;
    result.z = z - rhs.z;
    return result;
  }

  /** \brief operator overloading for the -= operator */
  Vector & operator-=(const Vector &rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  /** \brief operator overloading for the *= operator */
  Vector & operator*=(double rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }

  /** \brief operator overloading for the * operator */
  Vector  operator*(double rhs){
    Vector result;
    result.x = x*rhs;
    result.y = y*rhs;
    result.z = z*rhs;
    return result;
  }

  /** \brief Rotate a vector about the z-axis */
  Vector rot2D(double angle){
    Vector result;
    double cosa = cos(angle);
    double sina = sin(angle);
    result.x = cosa*x - sina*y;
    result.y = sina*x + cosa*y;
    result.z = z;
    return result;
  }

};
  
/** \brief A struct to represent the quaternion component */
struct Quaternion
{
  double x,y,z,w;

  /** \brief Constructor */
  Quaternion():x(0),y(0),z(0),w(1){;};
  /** \brief Constructor */
  Quaternion(double x, double y, double z, double w):x(x),y(y),z(z),w(w){;};

};
/** \brief A struct to represent Euler angles */
struct Euler
{
  double yaw, pitch, roll;
  /** \brief Constructor */
  Euler():yaw(0),pitch(0),roll(0){;};
  /** \brief Constructor */
  Euler(double yaw, double pitch, double roll):yaw(yaw),pitch(pitch),roll(roll){;};

};

/** \brief A class used to store and do basic minipulations of 3D transformations
   * 
   */
  class Pose3D 
  {
      friend class Pose3DCache;
      
  public:
  
      /* Constructors */
      /** \brief Empty Constructor initialize to zero */
      Pose3D();
      /** \brief Translation only constructor */
      Pose3D(double xt, double yt, double zt); 
      /** \brief  Quaternion only constructor */
      Pose3D(double xr, double yt, double zt, double w);
      /** \brief Translation and Quaternion constructor */
      Pose3D(Position &pos, Quaternion &quat);
      /** \brief Translation and Quaternion constructor */
      Pose3D(double xt, double yt, double zt, 
             double xr, double yr, double zr, double w);
      
      /** \brief Destructor */
      virtual ~Pose3D(void)
      {
      }
      
      // Utility functions to normalize and get magnitude.
      /** \brief Normalize the quaternion */
      void normalize(void);
      /** \brief Get the magnitude of the Quaternion 
       * used for normalization */
      double getMagnitude(void);
      /** \brief Assignment operator overload*/
      Pose3D & operator=(const Pose3D & input);
      /** \brief Assignment operator overload from Message*/
      Pose3D & operator=(const std_msgs::Pose3D & input);
  
    
      /* Accessors */
      /** \brief Return the transform as a matrix */
      NEWMAT::Matrix asMatrix() const;
      /** \brief Return the inverse of the transform as a matrix */
      NEWMAT::Matrix getInverseMatrix(void) const;
      /** \brief Return the rotation as a quaternion */
      Quaternion getQuaternion(void) const;
      /** \brief Return the rotation as a quaternion */
      void getQuaternion(Quaternion &quat) const;
      /** \brief Return the rotation as a Euler angles */
      Euler getEuler(void) const;
      /** \brief Return the rotation as a Euler angles */
      void getEuler(Euler &eu) const;
      /** \brief Return the translation as a position */
      void getPosition(Position &pos) const;
      /** \brief Return the translation as a position */
      Position   getPosition(void) const;
      /** \brief Return the rotation as an axis angle pair */
      void getAxisAngle(double axis[3], double *angle) const;
      
      /** \brief Get in ros message type */
      std_msgs::Pose3D getMessage(void) const;
    
      /** Mutators **/
      /** \brief Set the values to the identity transform */
      void setIdentity(void);      
      /** \brief Set the values from a matrix */
      void setFromMatrix(const NEWMAT::Matrix& matIn);
      /** \brief Set the values using Euler angles */
      void setFromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll);
      /** \brief Set the values using Euler angles */
      void setFromEuler(Position &pos, Euler &euler);
      /** \brief Set the values using DH Parameters */
      void setFromDH(double length, double alpha, double offset, double theta);
      /** \brief Set using ROS Message Type */
    void setFromMessage(const std_msgs::Pose3D& message);


      /** \brief Set the translational components */
      void setPosition(double x, double y, double z);
      /** \brief Set the translational components */
      void setPosition(Position &pos);
      /** \brief Set the rotational components */
      void setQuaternion(double x, double y, double z, double w);
      /** \brief Set the rotational components */
      void setQuaternion(Quaternion &quat);
      /** \brief Set the quaterion from an axis-angle representation */
      void setAxisAngle(double ax, double ay, double az, double angle);
      /** \brief Set the quaterion from an axis-angle representation */
      void setAxisAngle(double axis[3], double angle);
      
      /** \brief Set the translational components */
      void addPosition(double x, double y, double z);
      /** \brief Set the translational components */
      void addPosition(Position &pos);
      /** \brief Set the rotational components */
      void multiplyQuaternion(double x, double y, double z, double w);
      /** \brief Set the rotational components */
      void multiplyQuaternion(Quaternion &quat);
      
      /** \brief Apply another pose to the transform contained in the current pose (transform multiplication) */
      void multiplyPose(Pose3D &pose);
      
      /** \brief Invert this transfrom */
      void invert(void);      

      /** Application of the transform **/
      /** \brief Apply the stored transform to a point */
      void applyToPosition(Position &pos) const;
      /** \brief Apply the stored transform to a vector of points */
      void applyToPositions(std::vector<Position*> &posv) const;
      /** \brief Apply the stored transform to a point */
      void applyToVector(Vector &pos) const;
      /** \brief Apply the stored transform to a vector of points */
      void applyToVectors(std::vector<Vector*> &posv) const;
      
      /**************** Static Helper Functions ***********************/
      /** \brief Convert DH Parameters to a Homogeneous Transformation Matrix */
      static NEWMAT::Matrix matrixFromDH(double length, double alpha, double offset, double theta);
      /** \brief Convert Euler Angles to a Homogeneous Transformation Matrix */
      static NEWMAT::Matrix matrixFromEuler(double ax, double ay, double az,
					    double yaw, double pitch, double roll);
      /** \brief isolate Euler Angles from a homogenous transform matrix */
      static Euler    eulerFromMatrix(const NEWMAT::Matrix & matrix_in, unsigned int solution_number=1);
      /** \brief isolate translational change from a homogeneous transform matrix */
      static Position positionFromMatrix(const NEWMAT::Matrix & matrix_in);
      
  protected:
      
      /** Internal Data Storage*/    
      double xt, yt, zt, xr, yr, zr, w;

  };


  /** \brief A namespace ostream overload for displaying poses */
  std::ostream & operator<<(std::ostream& mystream, const Pose3D &pose);

  std::ostream & operator<<(std::ostream& mystream, const libTF::Vector &p);
 
}


#endif //LIBTF_POSE3D_HH
