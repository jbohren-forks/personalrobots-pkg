#include "CvStereoCamModel.h"

#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>
#include "CvMatUtils.h"


CvStereoCamModel::CvStereoCamModel(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
    this->constructProjectionMatrices();
}

CvStereoCamModel::CvStereoCamModel(CvStereoCamParams camParams):
    Parent(camParams),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
	constructProjectionMatrices();
}


CvStereoCamModel::CvStereoCamModel():
    Parent(),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
	constructProjectionMatrices();
}


CvStereoCamModel::~CvStereoCamModel()
{
}

void CvStereoCamModel::constructMat3DToScreen(double Fx, double Fy, double Tx,
		double Cx, double Cy, CvMat& mat){
    double data [] = {
        Fx, 0, Cx, -Fx*Tx,
        0, Fy, Cy, 0,
        0,  0,  1, 0
    };

    CvMat _P = cvMat(3, 4, CV_64FC1, data);

    cvCopy(&_P, &mat);
}

bool CvStereoCamModel::constructProjectionMatrices(){
    bool status = true;
    constructMat3DToScreen(mFx, mFy, mClx, mCy, mTx, mMatCartToScreenLeft);
    constructMat3DToScreen(mFx, mFy, mCrx, mCy, mTx, mMatCartToScreenRight);


    // construct the matrix that maps from Cartesian coordinates to
    // disparity coordinates
    double data [] = {
        mFx,   0,    mClx,        0,
        0,     mFy,  mCy,         0,
        0,     0,    mClx-mCrx,   mFx*mTx,
        0,     0,    1,           0
    };
    CvMat _P = cvMat(4, 4, CV_64FC1, data);
    cvCopy(&_P, &mMatCartToDisp);

 	// construct the matrix that maps from disparity coordinates to
	// Cartesian coordinates
#if 1
    // the following is computationally cleaner and may introduce less error
	double Q[] =
	{
		1,         0,          0,               -mClx,
		0,   mFx/mFy,          0,        -mCy*mFx/mFy,
		0,         0,          0,                 mFx,
		0,         0,      1/mTx,     -(mClx-mCrx)/mTx
	};
#else
	// the following form is more symmetrical, and goes along with
	// mCartToDisp
	double Q[] =
	{
		1./mFx,    0,          0,           -mClx/mFx,
		0,   	1./mFy,        0,           -mCy/mFy,
		0,         0,          0,                1,
		0,         0,      1./(mTx*mFx),    -(mClx-mCrx)/(mFx*mTx)
	};
#endif

	CvMat _Q = cvMat(4, 4, CV_64F, Q);
	cvCopy(&_Q, &mMatDispToCart);

	return status;
}

bool CvStereoCamModel::setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy){
    bool status = true;

    mFx  = Fx;
    mFy  = Fy;
    mClx = Clx;
    mCrx = Crx;
    mCy  = Cy;
    mTx  = Tx;

    status = this->constructProjectionMatrices();
    return status;
}

bool CvStereoCamModel::setCameraParams(const CvStereoCamParams& params) {
  double Fx, Fy, Tx, Clx, Crx, Cy;
  params.getParams(Fx, Fy, Tx, Clx, Crx, Cy);
  return setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
}

bool CvStereoCamModel::reprojection(const CvMat *uvds, CvMat *XYZs) const {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return dispToCart(*uvds, *XYZs);
}

bool CvStereoCamModel::dispToCart(const CvMat& uvds, CvMat & XYZs) const {
	bool status = true;

	CvMat uvds0;
	CvMat XYZs0;
	cvReshape(&uvds, &uvds0, 3, 0);
	cvReshape(&XYZs, &XYZs0, 3, 0);
	cvPerspectiveTransform(&uvds0, &XYZs0, &mMatDispToCart);
	return status;
}

//Id has to be  16SC1, Ixyz has to be 32F
bool CvStereoCamModel::dispToCart(const IplImage *Id, IplImage *Ixyz) const {
	bool status = true;
	cvReprojectImageTo3D(Id, Ixyz,&mMatDispToCart);
	return status;
}


bool CvStereoCamModel::projection(const CvMat *XYZs, CvMat *uvds) const {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return cartToDisp(*XYZs, *uvds);
}

bool CvStereoCamModel::cartToDisp(const CvMat& XYZs, CvMat& uvds) const {
	bool status = true;
	CvMat xyzs0;
	CvMat uvds0;
	cvReshape(&XYZs, &xyzs0, 3, 0);
	cvReshape(&uvds, &uvds0, 3, 0);
	cvPerspectiveTransform(&xyzs0, &uvds0, &mMatCartToDisp);
	return status;
}

