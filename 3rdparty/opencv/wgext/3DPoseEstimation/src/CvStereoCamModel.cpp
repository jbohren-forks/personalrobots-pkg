#include "CvStereoCamModel.h"

#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>


CvStereoCamModel::CvStereoCamModel(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy),
    mMat3DToScreenLeft(NULL),
    mMat3DToScreenRight(NULL),
    mMat3DToDisp(NULL)
{
    this->constructProjectionMatrices();
}

CvStereoCamModel::CvStereoCamModel(CvStereoCamParams camParams):
    Parent(camParams)
{
	constructProjectionMatrices();
}


CvStereoCamModel::CvStereoCamModel():
    Parent()
{
	constructProjectionMatrices();
}


CvStereoCamModel::~CvStereoCamModel()
{
}

CvMat* CvStereoCamModel::constructMat3DToScreen(double Fx, double Fy, double Tx, double Cx, double Cy){
    double data [] = {
        Fx, 0, Cx, -Fx*Tx,
        0, Fy, Cy, 0,
        0,  0,  1, 0
    };
    
    CvMat _P = cvMat(3, 4, CV_64FC1, data);
    
    return cvCloneMat(&_P);
}

bool CvStereoCamModel::constructProjectionMatrices(){
    bool status = true;
    this->mMat3DToScreenLeft  = constructMat3DToScreen(mFx, mFy, mClx, mCy, mTx);
    this->mMat3DToScreenRight = constructMat3DToScreen(mFx, mFy, mCrx, mCy, mTx);
    
    double data [] = {
        mFx,   0,    mClx,        0,
        0,     mFy,  mCy,         0,
        0,     0,    mClx-mCrx,   mFx*mTx,
        0,     0,    1,           0
    };
    CvMat _P = cvMat(4, 4, CV_64FC1, data);
    this->mMat3DToDisp = cvCloneMat(&_P);
    return status;
}

bool CvStereoCamModel::setCameraParameters(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy){
    bool status = true;
    
    mFx  = Fx;
    mFy  = Fy;
    mClx = Clx;
    mCrx = Crx;
    mCy  = Cy;
    mTx  = Tx;
    
    delete this->mMat3DToScreenLeft;
    delete this->mMat3DToScreenRight;
    delete this->mMat3DToDisp;
    
    status = this->constructProjectionMatrices();
    return status;
}

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
    cvPerspectiveTransform(&src0, &dst0, this->mMat3DToDisp);
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
    	cvMatMul(mMat3DToDisp, &src1, &dst1);

    	cvSetReal2D(dst, i, 0, dst1_data[0]/dst1_data[3]);
    	cvSetReal2D(dst, i, 1, dst1_data[1]/dst1_data[3]);
    	cvSetReal2D(dst, i, 2, dst1_data[2]/dst1_data[3]);
//    	cout<<"dst1: "<<dst1_data[0]<<","<<dst1_data[1]<<","<<dst1_data[2]<<","<<dst1_data[3]<<endl;
    }
    return status;
}

