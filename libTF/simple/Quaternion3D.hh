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
  //Constructor
  Quaternion3D(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w);

  inline void Set(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w)
  {xt = _xt; yt = _yt; zt = _zt; xr = _xr; yr = _yr; zr = _zr; w = _w;} ;

  void Normalize();
  double getMagnitude();

  NEWMAT::Matrix asMatrix();
  void printMatrix();


private:
  //Quaternion Storage
  double xt,yt,zt,xr,yr,zr,w;


};












#endif //QUATERNION3D_HH
