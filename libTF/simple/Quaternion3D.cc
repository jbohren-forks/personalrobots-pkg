#include "Quaternion3D.hh"

Euler3D::Euler3D(double _x, double _y, double _z, double _yaw, double _pitch, double _roll) :
  x(_x),y(_y),z(_z),yaw(_yaw),pitch(_pitch),roll(_roll)
{
  return;
};


Quaternion3D::Quaternion3D(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w):
  xt(_xt),yt(_yt),zt(_zt),xr(_xr),yr(_yr),zr(_zr),w(_w)
{
  Normalize();
  return;
};

Quaternion3D::Quaternion3D(NEWMAT::Matrix matIn)
{
  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html

  double * mat = matIn.Store();
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

      if ( mat[0] > mat[5] && mat[0] > mat[10] ) {// Column 0: 
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
};


void Quaternion3D::Normalize()
{
  double mag = getMagnitude();
  xr /= mag;
  yr /= mag;
  zr /= mag;
  w /= mag;
};

double Quaternion3D::getMagnitude()
{
  return sqrt(xr*xr + yr*yr + zr*zr + w*w);
};


NEWMAT::Matrix Quaternion3D::asMatrix()
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
  mat[4]  =     2 * ( xy - zw );
  mat[8]  =     2 * ( xz + yw );
  mat[1]  =     2 * ( xy + zw );
  mat[5]  = 1 - 2 * ( xx + zz );
  mat[9]  =     2 * ( yz - xw );
  mat[2]  =     2 * ( xz - yw );
  mat[6]  =     2 * ( yz + xw );
  mat[10] = 1 - 2 * ( xx + yy );
  mat[12]  = mat[13] = mat[14] = 0;
  mat[3] = xt;
  mat[7] = yt;
  mat[11] = zt;
  mat[15] = 1;
    

  return outMat;
};


void Quaternion3D::printMatrix()
{
  std::cout << asMatrix();

};
