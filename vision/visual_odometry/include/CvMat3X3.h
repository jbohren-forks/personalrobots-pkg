#ifndef WGMAT3X3_H_
#define WGMAT3X3_H_
#include <math.h>
#include <iostream>
using namespace std;

#include <opencv/cxcore.h>

#undef DEBUG

/** access of the (i,j)-th element of Mat, who has nCols columns */
#define ELEM(Mat, nCols, i, j)  Mat[(nCols)*(i)+(j)]

/** access to the (i,j)-th element of Mat, who has 3 columns */
#define ELEM3(Mat, i, j) ELEM(Mat, 3, i, j)
#define ELEM2(Mat, i, j) ELEM(Mat, 2, i, j)
/** dot product of the i-th row of A and j-th row of B */
#define DOTPROD3(A, i, B, j) ELEM3(A,i,0)*ELEM3(B,j,0)+ELEM3(A,i,1)*ELEM3(B,j,1)+ELEM3(A,i,2)*ELEM3(B,j,2)
#define DOTPROD3QQt(Q,i,j) DOTPROD3(Q,i,Q,j)
/** dot product of the i-th col of A and j-th col of B */
#define DOTPROD3COL(A, i, B, j) ELEM3(A,0, i)*ELEM3(B,0,j)+ELEM3(A,1,i)*ELEM3(B,1,j)+ELEM3(A,2,i)*ELEM3(B,2,j)
#define DOTPROD3QtQ(Q,i,j) DOTPROD3COL(Q,i,Q,j)

/** dot product of the i-th row of A and j-th row of B, both with 2 cols */
#define DOTPROD2(A, i, B, j) ELEM2(A,i,0)*ELEM2(B,j,0)+ELEM2(A,i,1)*ELEM2(B,j,1)


#define TRANSFORM_X(T, x, y, z) \
((T)[0]*(x) + (T)[1]*(y) + (T)[2]*(z) + (T)[3])

#define TRANSFORM_Y(T, x, y, z) \
((T)[4]*(x) + (T)[5]*(y) + (T)[6]*(z) + (T)[7])

#define TRANSFORM_Z(T, x, y, z) \
((T)[8]*(x) + (T)[9]*(y) + (T)[10]*(z) + (T)[11])

#define TRANSFORM_W(T, x, y, z) \
((T)[12]*(x) + (T)[13]*(y) + (T)[14]*(z) + (T)[15])

// [x1, y1, z1]^t = T * [x, y, z]^T
#define TRANSFORM(T, x, y, z, x1, y1, z1) \
	do {\
	(x1) = TRANSFORM_X(T, x, y, z); \
	(y1) = TRANSFORM_Y(T, x, y, z); \
	(z1) = TRANSFORM_Z(T, x, y, z); }while(0)

#define TRANSFORMRESIDUE(T, x0, y0, z0, x1, y1, z1, rx, ry, rz) \
do {(rx) = TRANSFORM_X(T, x0, y0, z0) - (x1); \
	(ry) = TRANSFORM_Y(T, x0, y0, z0) - (y1); \
	(rz) = TRANSFORM_Z(T, x0, y0, z0) - (z1);} while(0)

/// compute residue for perspective transformation
#define PERSTRANSFORMRESIDUE(T, x0, y0, z0, x1, y1, z1, rx, ry, rz) \
 do {double _s = 1.0/TRANSFORM_W(T, x0, y0, z0);\
  (rx) = TRANSFORM_X(T, x0, y0, z0)*_s - (x1); \
  (ry) = TRANSFORM_Y(T, x0, y0, z0)*_s - (y1); \
  (rz) = TRANSFORM_Z(T, x0, y0, z0)*_s - (z1);} while (0)

/// compute residue for perspective transformation
#define PERSTRANSFORMRESIDUE2(T, x0, y0, z0, x1, y1, z1, Tx_p, Ty_p, Tz_p, Tw_p, rx, ry, rz) \
 do {double _s = 1.0/(Tw_p=TRANSFORM_W(T, x0, y0, z0));\
  (rx) = (Tx_p=TRANSFORM_X(T, x0, y0, z0))*_s - (x1); \
  (ry) = (Ty_p=TRANSFORM_Y(T, x0, y0, z0))*_s - (y1); \
  (rz) = (Tz_p=TRANSFORM_Z(T, x0, y0, z0))*_s - (z1);} while (0)

/**
 * Template class for fast matrix operations over 3x3 matrices.
 */
template <typename DataT>
class CvMat3X3
{
public:
	CvMat3X3(){};
	virtual ~CvMat3X3(){};

	/**
	 * compute Q = A * B^T
	 * where A and B is 3x2 matrix.
	 * In one applications, Each of them stores
	 * 2 3D points in cols.
	 */
	static void AxBt3x2(DataT A[3*3], DataT B[6], DataT Q[3*3]) {
		ELEM3(Q, 0, 0) = DOTPROD2(A, 0, B, 0);
		ELEM3(Q, 0, 1) = DOTPROD2(A, 0, B, 1);
		ELEM3(Q, 0, 2) = DOTPROD2(A, 0, B, 2);

		ELEM3(Q, 1, 0) = DOTPROD2(A, 1, B, 0);
		ELEM3(Q, 1, 1) = DOTPROD2(A, 1, B, 1);
		ELEM3(Q, 1, 2) = DOTPROD2(A, 1, B, 2);

		ELEM3(Q, 2, 0) = DOTPROD2(A, 2, B, 0);
		ELEM3(Q, 2, 1) = DOTPROD2(A, 2, B, 1);
		ELEM3(Q, 2, 2) = DOTPROD2(A, 2, B, 2);
	};
	/// conventions of angle representations.
	typedef enum  {
		EulerXYZ,  //< The rotation is around X-axis first, then Y, then Z, i.e. Rx*Ry*Rz =>R
		EulerZYX,  //< The rotation is around Z-axis first, then Y, then X, i.e. Rz*Ry*Rx =>R
		Rodrigues  //< Rodrigues representation
	} AngleConvention;
	/**
	 * Construct the rotation matrix, given Euler angles
	 */
	static void rotMatrix(
      /// Euler angle for X-axis
	    DataT x,
      /// Euler angle for Y-axis
	    DataT y,
      /// Euler angle for Z-axis
	    DataT z,
	    /// (Output) constructed rotation matrix.
	    DataT R[3*3],
	    /// Convention of Euler angles
	    AngleConvention eulerAngleConvention=EulerZYX
	){
	  double cosz = cos(z);
	  double sinz = sin(z);
	  double _Rz[3*3] = { cosz, -sinz, 0.,
	      sinz,  cosz, 0.,
	      0.,    0., 1. };
	  double cosy = cos(y);
	  double siny = sin(y);
	  double _Ry[3*3] = { cosy, 0., -siny,
	      0., 1.,    0.,
	      siny, 0.,  cosy };

	  double cosx = cos(x);
	  double sinx = sin(x);

	  double _Rx[3*3] = { 1.,    0.,    0.,
	      0.,  cosx, -sinx,
	      0.,  sinx,  cosx };

	  CvMat Rx, Ry, Rz;
	  cvInitMatHeader(&Rx, 3, 3, CV_64FC1, _Rx);
	  cvInitMatHeader(&Ry, 3, 3, CV_64FC1, _Ry);
	  cvInitMatHeader(&Rz, 3, 3, CV_64FC1, _Rz);
	  double _Rxy[9], _Rxyz[9];
	  CvMat Rxy, Rxyz;
	  cvInitMatHeader(&Rxy,  3, 3, CV_64FC1, _Rxy);
	  cvInitMatHeader(&Rxyz, 3, 3, CV_64FC1, _Rxyz);
	  // TODO: would it be more efficient to follow symbolic computation
	  switch (eulerAngleConvention){
	  case EulerZYX:
	    // Rx*Ry*Rz =>R
	    cvMatMul(&Rx, &Ry, &Rxy);
	    cvMatMul(&Rxy, &Rz, &Rxyz);
	    break;
	  case EulerXYZ:
	    // Rz*Ry*Rx =>R
	    cvMatMul(&Ry, &Rx, &Rxy);
	    cvMatMul(&Rz, &Rxy, &Rxyz);
	    break;
	  default:
	    cerr <<"Not implemented yet"<<endl;
	    return;
	  }
	  // copy the result into R[]
	  for (int i=0; i<3; i++) {
	    for (int j=0; j<3; j++) {
	      R[i*3+j] = cvmGet(&Rxyz, i, j);
	    }
	  }
#if DEBUG==1
	  cout << "CvMat3x3::rotMatrix()"<<endl;
	  printMat(R);
	  cout << "Qx:"<< endl;
	  printMat(_Rx);
	  cout << "Qy:"<<endl;
	  printMat(_Ry);
	  cout << "Qz:"<<endl;
	  printMat(_Rz);
#endif
	};
	/**
	 * Construct transformation matrix given the euler angles
	 * and translation vectors.
	 */
	inline static void transformMatrix(
	    /// Euler angle for X-axis
	    DataT x,
	    /// Euler angle for Y-axis
	    DataT y,
	    /// Euler angle for Z-axis
	    DataT z,
	    /// translation in X-axis
	    DataT tx,
	    /// translation in Y-axis
	    DataT ty,
	    /// translation in Z-axis
	    DataT tz,
	    /// (Output) transformation matrix
	    DataT RT[],
	    /// number of columns in RT.
	    int numCols,
	    /// Euler angle convention.
	    AngleConvention eulerAngleConvention=EulerZYX
	){
	  DataT rot[3*3];
	  rotMatrix(x, y, z, rot, eulerAngleConvention);
	  for (int i=0; i<3; i++){
	    for (int j=0; j<3; j++) {
	      RT[i*numCols + j] = rot[i*3+j];
	    }
	  }
	  RT[0*numCols + 3] = tx;
	  RT[1*numCols + 3] = ty;
	  RT[2*numCols + 3] = tz;
	}
	static void transformMatrixSq(DataT x, DataT y, DataT z,
	    DataT tx, DataT ty, DataT tz,
	    DataT RT[], int numCols,
	    AngleConvention eulerAngleConvention=EulerZYX
	){
	  transformMatrix(x, y, z, tx, ty, tz, RT, numCols, eulerAngleConvention);
	  // last row;
	  RT[3*numCols + 0] = 0.0;
	  RT[3*numCols + 1] = 0.0;
	  RT[3*numCols + 2] = 0.0;
	  RT[3*numCols + 3] = 1.0;
	}
	static void printMat(DataT mat[]){
	  for (int i=0; i<3; i++) {
	    for (int j=0; j<3; j++) {
	      printf("%12.5f,", mat[i*3+j]);
	    }
	    cout << endl;
	  }
	}
private:
	static int chooseGoodDiagElem(DataT A[]){
		return 0;
	}
	static int chooseGoodDiagMinor(DataT A[], int excludedIndex){
		return 1;
	}
};

#endif /*WGMAT3X3_H_*/
