#include "CvStereoCamModel.h"

#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>


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
	// cartesian coordinates
#if 0
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

#if 0 // TODO: delete it. Seems to be duplicated
bool CvStereoCamModel::convert3DToDisparitySpace(CvMat* src, CvMat* dst) {
    bool status = true;
    int numPoints = src->rows;
    if (numPoints != dst->rows) {
        cerr << "mismatch numbers of points in src and dst"<<endl;
        return false;
    }

#if 0
    CvMat src0, dst0;
    cvReshape(src, &src0, 3, 0);
    cvReshape(dst, &dst0, 3, 0);
    cvPerspectiveTransform(&src0, &dst0, this->mMatCartToDisp);
#endif
    double dst1_data[4];
    CvMat src1;
    CvMat dst1 = cvMat(4, 1, CV_64FC1, dst1_data);

    for (int i=0; i<numPoints; i++) {
    	double data []= {
    			cvmGet(src, i, 0),
    			cvmGet(src, i, 1),
    			cvmGet(src, i, 2),
    			1.0,
    	};
    	cvInitMatHeader(&src1, 4, 1, CV_64FC1, data);
    	cvMatMul(&mMatCartToDisp, &src1, &dst1);

    	cvSetReal2D(dst, i, 0, dst1_data[0]/dst1_data[3]);
    	cvSetReal2D(dst, i, 1, dst1_data[1]/dst1_data[3]);
    	cvSetReal2D(dst, i, 2, dst1_data[2]/dst1_data[3]);
//    	cout<<"dst1: "<<dst1_data[0]<<","<<dst1_data[1]<<","<<dst1_data[2]<<","<<dst1_data[3]<<endl;
    }
    return status;
}
#endif

bool CvStereoCamModel::reprojection(const CvMat *uvds, CvMat *XYZs) {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return dispToCart(*uvds, *XYZs);
}

bool CvStereoCamModel::dispToCart(const CvMat& XYZs, CvMat & uvds) {
	bool status = true;

	CvMat uvds0;
	CvMat XYZs0;
	cvReshape(&uvds, &uvds0, 3, 0);
	cvReshape(&XYZs, &XYZs0, 3, 0);
	cvPerspectiveTransform(&uvds0, &XYZs0, &mMatDispToCart);
	return status;
}

bool CvStereoCamModel::projection(const CvMat *XYZs, CvMat *uvds) {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return cartToDisp(*XYZs, *uvds);
}

bool CvStereoCamModel::cartToDisp(const CvMat& XYZs, CvMat& uvds) {
	bool status = true;
	CvMat xyzs0;
	CvMat uvds0;
	cvReshape(&XYZs, &xyzs0, 3, 0);
	cvReshape(&uvds, &uvds0, 3, 0);
	cvPerspectiveTransform(&xyzs0, &uvds0, &mMatCartToDisp);
	return status;
}

