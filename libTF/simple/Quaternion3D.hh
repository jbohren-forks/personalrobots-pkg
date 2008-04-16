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
