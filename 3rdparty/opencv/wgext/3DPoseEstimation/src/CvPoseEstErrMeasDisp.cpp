/*
 * CvPoseEstErrMeasDisp.cpp
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#include <iostream>
using namespace std;

#include <opencv/cxcore.h>

#include "CvPoseEstErrMeasDisp.h"
#include "Cv3DPoseEstimateDisp.h"
#include "CvMatUtils.h"
using namespace cv::willow;

CvPoseEstErrMeasDisp::CvPoseEstErrMeasDisp():
	mHomography(cvMat(4, 4, CV_64FC1, _mHomography)){
}

CvPoseEstErrMeasDisp::~CvPoseEstErrMeasDisp() {
}

void CvPoseEstErrMeasDisp::transform(const CvMat& src, CvMat& dst){
	CvMat srcC3;
	CvMat dstC3;
	cvReshape(&src, &srcC3, 3, 0);
	cvReshape(&dst, &dstC3, 3, 0);
	cvPerspectiveTransform(&srcC3, &dstC3, &mHomography);
}


// compute the error in disparity space
void CvPoseEstErrMeasDisp::measure(const CvMat& uvds0, const CvMat& uvds1) {
#ifdef DEBUG
	cout << "Entering "<< __PRETTY_FUNCTION__ <<endl;
#endif

	int n = uvds0.rows;
	double _xyzs0[3*n], _xyzs1[3*n];
	CvMat xyzs0 = cvMat(n, 3, CV_64FC1, _xyzs0);
	CvMat xyzs1 = cvMat(n, 3, CV_64FC1, _xyzs1);

	dispToCart(uvds0, xyzs0);
	dispToCart(uvds1, xyzs1);

	// compare in Cartesian Space
	CvPoseEstErrMeas::compare(xyzs0, xyzs1);

	// compare in Disparity Space
	Cv3DPoseEstimateDisp::constructHomography(mRotation, mShift, mMatDispToCart, mMatCartToDisp,
			mHomography);

	double _uvds11[3*n];
	CvMat uvds11 = cvMat(n, 3, CV_64FC1, _uvds11);
	this->transform(uvds0, uvds11);

	this->compare(uvds11, uvds1);
}

void CvPoseEstErrMeasDisp::compare(const CvMat& uvds11, const CvMat& uvds1){
	cout << "entering "<<__PRETTY_FUNCTION__<<endl;
	int n = uvds1.rows; // num of points;

	CvMat uvds11_img;
	CvMat uvds1_img;
	cvReshape(&uvds11, &uvds11_img, 3, 0);
	cvReshape(&uvds1,  &uvds1_img,  3, 0);
//	double err = cvNorm(&uvds11, &uvds1, CV_L2)/n;

	double _uvsRight1[2*n];
	double _uvsRight11[2*n];
	double _uvdToUvRight[] = {
			 1., 0.,
			 0., 1.,
			-1., 0.
	};
	double _uvds1AbsDiff[3*n];
	double _uvLeftAbsDiff[2*n];
	double _uvRightAbsDiff[2*n];
	double _uvds1Diff[3*n];
	CvMat uvsLeft1;  cvGetCols(&uvds1,  &uvsLeft1,  0, 2);
	CvMat uvsLeft11; cvGetCols(&uvds11, &uvsLeft11, 0, 2);
	CvMat uvsRight1  = cvMat(n, 2, CV_64FC1, _uvsRight1);
	CvMat uvsRight11 = cvMat(n, 2, CV_64FC1, _uvsRight11);

	const CvMat uvdToUvRight = cvMat(3, 2, CV_64FC1, _uvdToUvRight);
	cvMatMul(&uvds1,   &uvdToUvRight, &uvsRight1);
	cvMatMul(&uvds11, &uvdToUvRight, &uvsRight11);

	CvMat uvds1AbsDiff   = cvMat(n, 3, CV_64F, _uvds1AbsDiff);
	CvMat uvds1Diff      = cvMat(n, 3, CV_64F, _uvds1Diff);
	CvMat uvLeftAbsDiff  = cvMat(n, 2, CV_64FC1, _uvLeftAbsDiff);
	CvMat uvRightAbsDiff = cvMat(n, 2, CV_64FC1, _uvRightAbsDiff);

	cvAbsDiff(&uvds11, &uvds1, &uvds1AbsDiff);
	cvSub(&uvds11, &uvds1, &uvds1Diff);
	cvAbsDiff(&uvsLeft11,  &uvsLeft1,  &uvLeftAbsDiff);
	cvAbsDiff(&uvsRight11, &uvsRight1, &uvRightAbsDiff);

	double _errL2 = 0;
	double errLeftL2 = 0;
	double errRightL2 = 0;
	for (int k=0; k<n; k++){
		_errL2 += cvSqrt(
				_uvds1AbsDiff[3*k  ]*_uvds1AbsDiff[3*k  ] +
				_uvds1AbsDiff[3*k+1]*_uvds1AbsDiff[3*k+1] +
				_uvds1AbsDiff[3*k+2]*_uvds1AbsDiff[3*k+2]);

		errLeftL2 += cvSqrt(
				_uvLeftAbsDiff[2*k]*_uvLeftAbsDiff[2*k] +
				_uvLeftAbsDiff[2*k+1]*_uvLeftAbsDiff[2*k+1]);
		errRightL2 += cvSqrt(
				_uvRightAbsDiff[2*k]*_uvRightAbsDiff[2*k] +
				_uvRightAbsDiff[2*k+1]*_uvRightAbsDiff[2*k+1]);
	}
	_errL2     /= n;
	errLeftL2  /= n;
	errRightL2 /= n;

//	double errL2 = cvNorm(&uvds11_img, &uvds1_img, CV_L2)/n;
	double errL1 = cvNorm(&uvds11_img, &uvds1_img, CV_L1)/n;
	double errC  = cvNorm(&uvds11_img, &uvds1_img, CV_C);

	cout << "Errors in Disparity Space"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errC   <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errL1  <<endl;
	cout << "Average L-2 error in pixels of all corners: " << _errL2 <<endl;

	double errLeftL1  = cvNorm(&uvLeftAbsDiff,  NULL, CV_L1)/n;
	double errRightL1 = cvNorm(&uvRightAbsDiff, NULL, CV_L1)/n;
	double errLeftC   = cvNorm(&uvLeftAbsDiff,  NULL, CV_C);
	double errRightC  = cvNorm(&uvRightAbsDiff, NULL, CV_C);

	cout << "Errors in Screen Spaces (Left, Right)"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errLeftC  <<","<< errRightC <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errLeftL1 <<","<<errRightL1 <<endl;
	cout << "Average L-2 error in pixels of all corners: " << errLeftL2 <<","<<errRightL2 << endl;

}

#if 1
void CvPoseEstErrMeasDisp::measureMixed(const CvMat& xyzs0, const CvMat& uvds1) {
	cout << "entering "<<__PRETTY_FUNCTION__<<endl;
	int n = xyzs0.rows; // num of points;
	// transform xyz0 into xyz1 using the estimated rot and shift matrices
	double _xyzs1[3*n];
	double _uvds0[3*n];
	CvMat xyzs1 = cvMat(n, 3, CV_64F, _xyzs1);
	CvMat uvds0 = cvMat(n, 3, CV_64F, _uvds0);

	this->cartToDisp(xyzs0, uvds0);
	this->measure(uvds0, uvds1);

	this->dispToCart(uvds1, xyzs1);

	this->measure(xyzs0, xyzs1);
}

#else
void CvPoseEstErrMeasDisp::measureMixed2(CvMat& xyzs0, CvMat& uvds1) {
	cout << "entering "<<__PRETTY_FUNCTION__<<endl;
	int n = xyzs0.rows; // num of points;
	// transform xyz0 into xyz1 using the estimated rot and shift matrices
	double _xyzs11[3*n];
	double _uvds11[3*n];
	CvMat xyzs11 = cvMat(n, 3, CV_64F, _xyzs11);

	// transform the point cloud of pose 0 to pose 1
	CvPoseEstErrMeas::transform(xyzs0, xyzs11);

	CvMat uvds11 = cvMat(n, 3, CV_64F, _uvds11);

	projection(&xyzs11, &uvds11);

	CvMat uvds11_img;
	CvMat uvds1_img;
	cvReshape(&uvds11, &uvds11_img, 3, 0);
	cvReshape(&uvds1,  &uvds1_img,  3, 0);
//	double err = cvNorm(&uvds11, &uvds1, CV_L2)/n;

	double _uvsRight1[2*n];
	double _uvsRight11[2*n];
	double _uvdToUvRight[] = {
			 1., 0.,
			 0., 1.,
			-1., 0.
	};
	double _uvds1AbsDiff[3*n];
	double _uvLeftAbsDiff[2*n];
	double _uvRightAbsDiff[2*n];
	double _uvds1Diff[3*n];
	CvMat uvsLeft1;  cvGetCols(&uvds1,  &uvsLeft1,  0, 2);
	CvMat uvsLeft11; cvGetCols(&uvds11, &uvsLeft11, 0, 2);
	CvMat uvsRight1  = cvMat(n, 2, CV_64FC1, _uvsRight1);
	CvMat uvsRight11 = cvMat(n, 2, CV_64FC1, _uvsRight11);

	const CvMat uvdToUvRight = cvMat(3, 2, CV_64FC1, _uvdToUvRight);
	cvMatMul(&uvds1,   &uvdToUvRight, &uvsRight1);
	cvMatMul(&uvds11, &uvdToUvRight, &uvsRight11);

	CvMat uvds1AbsDiff   = cvMat(n, 3, CV_64F, _uvds1AbsDiff);
	CvMat uvds1Diff      = cvMat(n, 3, CV_64F, _uvds1Diff);
	CvMat uvLeftAbsDiff  = cvMat(n, 2, CV_64FC1, _uvLeftAbsDiff);
	CvMat uvRightAbsDiff = cvMat(n, 2, CV_64FC1, _uvRightAbsDiff);

	cvAbsDiff(&uvds11, &uvds1, &uvds1AbsDiff);
	cvSub(&uvds11, &uvds1, &uvds1Diff);
	cvAbsDiff(&uvLeftAbsDiff,  &uvsLeft11,  &uvsLeft1);
	cvAbsDiff(&uvRightAbsDiff, &uvsRight11, &uvsRight1);

	double _errL2 = 0;
	double errLeftL2 = 0;
	double errRightL2 = 0;
	for (int k=0; k<n; k++){
		_errL2 += cvSqrt(
				_uvds1AbsDiff[3*k  ]*_uvds1AbsDiff[3*k  ] +
				_uvds1AbsDiff[3*k+1]*_uvds1AbsDiff[3*k+1] +
				_uvds1AbsDiff[3*k+2]*_uvds1AbsDiff[3*k+2]);

		errLeftL2 += cvSqrt(
				_uvLeftAbsDiff[2*k]*_uvLeftAbsDiff[2*k] +
				_uvLeftAbsDiff[2*k+1]*_uvLeftAbsDiff[2*k+1]);
		errRightL2 += cvSqrt(
				_uvRightAbsDiff[2*k]*_uvRightAbsDiff[2*k] +
				_uvRightAbsDiff[2*k+1]*_uvRightAbsDiff[2*k+1]);
	}
	_errL2     /= n;
	errLeftL2  /= n;
	errRightL2 /= n;

//	double errL2 = cvNorm(&uvds11_img, &uvds1_img, CV_L2)/n;
	double errL1 = cvNorm(&uvds11_img, &uvds1_img, CV_L1)/n;
	double errC  = cvNorm(&uvds11_img, &uvds1_img, CV_C);

	cout << "Errors in Disparity Space"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errC   <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errL1  <<endl;
	cout << "Average L-2 error in pixels of all corners: " << _errL2 <<endl;

	double errLeftL1  = cvNorm(&uvLeftAbsDiff,  NULL, CV_L1)/n;
	double errRightL1 = cvNorm(&uvRightAbsDiff, NULL, CV_L1)/n;
	double errLeftC   = cvNorm(&uvLeftAbsDiff,  NULL, CV_C);
	double errRightC  = cvNorm(&uvRightAbsDiff, NULL, CV_C);

	cout << "Errors in Screen Spaces (Left, Right)"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errLeftC<<","<< errRightC <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errLeftL1 <<","<<errRightL1 <<endl;
	cout << "Average L-2 error in pixels of all corners: " << errLeftL2<<","<<errRightL2 << endl;

	double _xyzs1[3*n];
	CvMat xyzs1 = cvMat(n, 3, CV_64F, _xyzs1);
	reprojection(&uvds1, &xyzs1);

//	double err3DL2 = cvNorm(&xyzs11, &xyzs1, CV_L2)/n;
	double err3DL1 = cvNorm(&xyzs11, &xyzs1, CV_L1)/n;
	double err3DC  = cvNorm(&xyzs11, &xyzs1, CV_C);

#if 0
	cout << "Errors in Cartesian Space"<<endl;
	cout << "Inf norm of error in mm of all corners: " << err3DC <<endl;
	cout << "Average L-1 error in mm of all corners: " << err3DL1 <<endl;
	cout << "Average L-2 error in mm of all corners: " << err3DL2 <<endl;
#endif

	double _xyzs1AbsDiff[3*n];
	double _xyzs1Diff[3*n];
	CvMat xyzs1AbsDiff = cvMat(n, 3, CV_64F, _xyzs1AbsDiff);
	CvMat xyzs1Diff = cvMat(n, 3, CV_64F, _xyzs1Diff);
	cvAbsDiff(&xyzs11, &xyzs1, &xyzs1AbsDiff);
	// compute the diff vectors from xyzs1 to xyzs11
	cvSub(&xyzs11, &xyzs1, &xyzs1Diff);

	double _err3DL2 = 0;
	for (int k=0; k<n; k++){
		_err3DL2 += cvSqrt(
		_xyzs1AbsDiff[3*k  ]*_xyzs1AbsDiff[3*k  ] +
		_xyzs1AbsDiff[3*k+1]*_xyzs1AbsDiff[3*k+1] +
		_xyzs1AbsDiff[3*k+2]*_xyzs1AbsDiff[3*k+2]);
	}
	_err3DL2 /= n;
//	CvScalar sum = cvSum(&xyzs1AbsDiff);
//	double _err3DL1 = (sum.val[0]+sum.val[1]+sum.val[2])/n;
//	double _err3DC  = cvNorm(&xyzs1AbsDiff, NULL, CV_C);

	cout << "Errors  in Cartesian Space"<<endl;
	cout << "Inf norm of error in mm of all corners: " << err3DC <<endl;
	cout << "Average L-1 error in mm of all corners: " << err3DL1 <<endl;
	cout << "Average L-2 error in mm of all corners: " << _err3DL2 <<endl;



#if 0
	cout << "uvds11, and uvds1, interlaced"<<endl;
	for (int i=0; i<n; i++){
		cout << "est: "<< _uvds11[3*i]<<" "<<_uvds11[3*i+1]<<" "<<_uvds11[3*i+2]<<" "<< endl;
		cout << "msr: "<<_uvds1[3*i] <<" "<<_uvds1[3*i+1]<<" "<<_uvds1[3*i+2]<<" "<< endl;
	}
#endif


	// measure errors in x-y and z (depth) separately, as the lrf tends to have larger error
	// in depth
	CvMat xy1AbsDiff;
	CvMat z1AbsDiff;

	cvGetCols(&xyzs1AbsDiff, &xy1AbsDiff, 0, 1);
	cvGetCol(&xyzs1AbsDiff,  &z1AbsDiff,  2);

	double errC_3DXY = cvNorm(&xy1AbsDiff, NULL, CV_C);
	double errC_3DZ  = cvNorm(&z1AbsDiff,  NULL, CV_C);

	double errL1_3DXY = cvNorm(&xy1AbsDiff, NULL, CV_L1)/n;
	double errL1_3DZ  = cvNorm(&z1AbsDiff, NULL, CV_L1)/n;

	double errL2_3DXY = 0;
	double errL2_3DZ  = 0;
	for (int k=0; k<n; k++){
		errL2_3DXY += cvSqrt(
		_xyzs1AbsDiff[3*k  ]*_xyzs1AbsDiff[3*k  ] +
		_xyzs1AbsDiff[3*k+1]*_xyzs1AbsDiff[3*k+1]);

		errL2_3DZ += _xyzs1AbsDiff[3*k+2];
	}
	errL2_3DXY /= n;
	errL2_3DZ  /= n;

	cout << "Errors  in Cartesian Space (measured separately in X-Y and Z"<<endl;
	cout << "Inf norm of error in mm of all corners: " << errC_3DXY << ","<<errC_3DZ <<endl;
	cout << "Average L-1 error in mm of all corners: " << errL1_3DXY << ","<<errL1_3DZ <<endl;
	cout << "Average L-2 error in mm of all corners: " << errL2_3DXY << ","<<errL2_3DZ <<endl;

	// measure bias in cartesian space
	// simply compute the average of the error vectors of each data point
	CvMat xyzs1Diff_3c;
	cvReshape(&xyzs1Diff, &xyzs1Diff_3c, 3, 0);
	CvScalar avgErrorVector = cvAvg(&xyzs1Diff_3c);

	cout << "Average error vector:"<<avgErrorVector.val[0]<<","<<avgErrorVector.val[1]<<","<<avgErrorVector.val[2]<<endl;
}
#endif
