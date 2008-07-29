#include <iostream.h>
#include <cxtypes.h>
#include "calib_stereo.h"
#include <cv.h>
#include "Cv3DPoseEstimateDispSpaceRef.h"
#include "CvStereoCamModel.h"
#include "CvLevMarqDispSpace.h"
#include "CvMat3X3.h"
#include "CvMatUtils.h"

#include "CvTestTimer.h"

//#define DEBUG

//#define DEBUG_DISTURB_PARAM

#if 0
#define TIMERSTART(x) 
#define TIMEREND(x)
#define TIMERSTART2(x) 
#define TIMEREND2(x)
#else
#define TIMERSTART(x) CvTestTimerStart(x)
#define TIMEREND(x) CvTestTimerEnd(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x) CvTestTimerEnd2(x)
#endif


Cv3DPoseEstimateDispSpaceRef::Cv3DPoseEstimateDispSpaceRef():
	PoseParent(),
	Parent(),
	//mProjectionMat(NULL), mInvProjectionMat(NULL), 
	mDispToCart(NULL), mCartToDisp(NULL)
{
	// overide parent default
	mErrThreshold = 1.5;
}

Cv3DPoseEstimateDispSpaceRef::~Cv3DPoseEstimateDispSpaceRef()
{
	// TODO: remove projection mat and reprojection mat
}

int Cv3DPoseEstimateDispSpaceRef::estimate(
		CvMat *xyzs0, CvMat *uvds1, CvMat *refPoints0, CvMat *refPoints1,
		CvMat *rot, CvMat *shift, CvMat *& inliers0, CvMat *& outliers1){
	int numInLiers = 0;
	
	// convert the first point cloud, xyzs0, which is in cartesian space
	// into disparity space
	int numPoints = xyzs0->rows;
	
	CvMat* uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
	CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);
	
	reprojection(uvds1, xyzs1);
	projection(xyzs0, uvds0);
	
	numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1, 
			refPoints0, refPoints1,
			rot, shift, inliers0, outliers1);
	
	cvReleaseMat(&uvds0);
	cvReleaseMat(&xyzs1);
	return numInLiers;
}
	


int Cv3DPoseEstimateDispSpaceRef::estimate(CvMat *uvds0, CvMat *uvds1,
		CvMat *rot, CvMat *shift, CvMat*& outliers0, CvMat*& outliers1) {
	int numInLiers = 0;
	
	int numPoints = uvds0->rows;
	
	if (numPoints != uvds1->rows) {
		cerr << "number of points mismatched in input" << endl;
		return 0;
	}
	
	CvMat* xyzs0 = cvCreateMat(numPoints, 3, CV_64FC1);
	CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);
	
	// project from disparity space back to XYZ
	reprojection(uvds0, xyzs0);
	reprojection(uvds1, xyzs1);
	
	numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1, NULL, NULL, rot, shift,
			outliers0, outliers1);
	
	cvReleaseMat(&xyzs0);
	cvReleaseMat(&xyzs1);
	return numInLiers;
}
	
int Cv3DPoseEstimateDispSpaceRef::estimate(CvMat *xyzs0, CvMat *xyzs1,
			CvMat *uvds0, CvMat *uvds1, CvMat *refPoints0, CvMat *refPoints1,
			CvMat *rot, CvMat *shift, CvMat *& inliers0, CvMat *& inliers1)
{
	int numInLiers = 0;
	double _P0[3*3], _P1[3*3], _R[3*3], _T[3*1], _H[4*4];
	CvMat P0, P1;
	cvInitMatHeader(&P0, 3, 3, CV_64FC1, _P0); // 3 random points from camera0, stored in columns
	cvInitMatHeader(&P1, 3, 3, CV_64FC1, _P1); // 3 random points from caemra1, stored in columns

	CvMat R, T, H;
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);  // rotation matrix, R = V * tranpose(U)
	cvInitMatHeader(&T,  3, 1, CV_64FC1, _T);  // translation matrix
	cvInitMatHeader(&H,  4, 4, CV_64FC1, _H);  // disparity space homography
	
	int maxNumInLiers=0;
	if (refPoints0 ==  NULL || refPoints1 == NULL) {
		for (int i=0; i< mNumIterations; i++) {
#ifdef DEBUG
			cout << "Iteration: "<< i << endl;
#endif
			// randomly pick 3 points. make sure they are not 
			// colinear
			pick3RandomPoints(xyzs0, xyzs1, &P0, &P1);

			TIMERSTART2(SVD);
			this->estimateLeastSquareInCol(&P0, &P1, &R, &T);
			TIMEREND2(SVD);

			this->constructDisparityHomography(&R, &T, &H);

			CvTestTimerStart(CheckInliers);
			// scoring against all points
			numInLiers = checkInLiers(uvds0, uvds1, &H);
			CvTestTimerEnd(CheckInliers);

#ifdef DEBUG
			cout << "R, T, and H: "<<endl;
			CvMatUtils::printMat(&R);
			CvMatUtils::printMat(&T);
			CvMatUtils::printMat(&H);
#endif

			// keep the best R and T
			if (maxNumInLiers < numInLiers) {
				maxNumInLiers = numInLiers;
				cvCopy(&R, rot);
				cvCopy(&T, shift);
			}
		}
	} else {
		this->estimateLeastSquareInCol(refPoints0, refPoints1, rot, shift);
		this->constructDisparityHomography(rot, shift, &H);
		maxNumInLiers = checkInLiers(uvds0, uvds1, &H);
	}
	
	if (maxNumInLiers<6) {
		cout << "Too few inliers: "<< maxNumInLiers << endl;
		return maxNumInLiers;
	}
    
    // get a copy of all the inliers, original and transformed
    CvMat *uvds0Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    CvMat *uvds1Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    // construct homography matrix
    constructDisparityHomography(rot, shift, &H);
    int numInliers0 = getInLiers(uvds0, uvds1, &H, uvds0Inlier, uvds1Inlier);
	
    // make a copy of the best Transformation before nonlinear optimization
    cvCopy(&H, &mRTBestWithoutLevMarq);
    
    
	// get the euler angle from rot
	CvPoint3D64f eulerAngles;
	{
		double _R[9], _Q[9];
        CvMat R, Q;
        CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
        cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
        cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

#ifdef DEBUG // all debugging stuff        
        double _Qx[9], _Qy[9], _Qz[9], _Qyz[9], _Q1T[9];
        CvMat Qx, Qy, Qz, Qyz, Q1T;  // optional. For debugging.
        double _Q1[9];
        CvMat Q1;
        cvInitMatHeader(&Qx, 3, 3, CV_64FC1, _Qx);
        cvInitMatHeader(&Qy, 3, 3, CV_64FC1, _Qy);
        cvInitMatHeader(&Qz, 3, 3, CV_64FC1, _Qz);
        cvInitMatHeader(&Qyz, 3, 3, CV_64FC1, _Qyz);
        cvInitMatHeader(&Q1T, 3, 3, CV_64FC1, _Q1T);
        cvInitMatHeader(&Q1, 3, 3, CV_64FC1, _Q1);
        pQx = &Qx;
        pQy = &Qy;
        pQz = &Qz;
#endif
	
        cvRQDecomp3x3((const CvMat*)rot, &R, &Q, pQx, pQy, pQz, &eulerAngles);
        
#ifdef DEBUG
        cout << "RQ decomposition rot => R, Q"<< endl;
        CvMatUtils::printMat(rot);
        CvMatUtils::printMat(&R);
        CvMatUtils::printMat(&Q);
        cout << "reconstruct rot matrices. Qx, Qy and Qz"<<endl;
        CvMatUtils::printMat(&Qx);
        CvMatUtils::printMat(&Qy);
        CvMatUtils::printMat(&Qz);
        cout <<"euler angles"<<endl;
        cout << eulerAngles.x<<","<<eulerAngles.y<<","<<eulerAngles.z<<endl;
        cvGEMM(&Qy, &Qz, 1, NULL, 0, &Qyz);
        cvGEMM(&Qx, &Qyz, 1, NULL, 0, &Q1T);
        cvTranspose(&Q1T, &Q1);
        cout <<"(Qx * Qy * Qz)^T => Q1: "<< endl;
        CvMatUtils::printMat(&Q1);
#endif 
	}
	
    // nonlinear optimization by Levenberg-Marquardt
	CvLevMarqTransformDispSpace levMarq(this->mDispToCart, this->mCartToDisp, 
        numInliers0);
    
    double param[6];
    
    //initialize the parameters
    param[0] = eulerAngles.x/180. * CV_PI;
    param[1] = eulerAngles.y/180. * CV_PI;
    param[2] = eulerAngles.z/180. * CV_PI;
    param[3] = cvmGet(shift, 0, 0);
    param[4] = cvmGet(shift, 1, 0);
    param[5] = cvmGet(shift, 2, 0);
    
#ifdef DEBUG
    printf("initial parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
    		param[3], param[4], param[5]);
#endif
    
#ifdef    DEBUG_DISTURB_PARAM
    for (int i=0; i<6; i++) param[i] *= 1.01;
    printf("disturbed parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
    		param[3], param[4], param[5]);
#endif

    CvTestTimerStart(LevMarqDoit);
    levMarq.optimize(uvds0Inlier, uvds1Inlier, param);    
    CvTestTimerEnd(LevMarqDoit);
    
#ifdef DEBUG
    printf("optimized parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
    		param[3], param[4], param[5]);
#endif
    
	// TODO: construct matrix with parameters from nonlinear optimization
    double _rot[9];
    
    CvMat3X3<double>::rotMatrix(param[0], param[1], param[2], _rot,
    		CvMat3X3<double>::XYZ);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) {
            cvmSet(rot, i, j, _rot[i*3+j]);
        }
    cvmSet(shift, 0, 0, param[3]);
    cvmSet(shift, 1, 0, param[4]);
    cvmSet(shift, 2, 0, param[5]);
    
    inliers0 = uvds0Inlier;
    inliers1 = uvds1Inlier;
    
    // construct the final transformation matrix
    this->constructDisparityHomography(rot, shift, &mT);
	return numInliers0;
}

bool Cv3DPoseEstimateDispSpaceRef::constructDisparityHomography(CvMat *R, CvMat *T,
    CvMat *H){
    bool status = true;
    // Transformation matrix RT:
    //         R  t
    // RT = (       )
    //         0  1
    // Disparity Space Homography
    // H = Gamma RT inv(Gamma)
    //
    
    double _RT[16], _G[16];
    CvMat RT, G;    
    cvInitMatHeader(&RT, 4, 4, CV_64FC1, _RT); // transformation matrix (including rotation and translation)
    cvInitMatHeader(&G,  4, 4, CV_64FC1, _G);  // a temp matrix 
    
    constructRT(R, T, &RT);
     
    cvMatMul(mCartToDisp, &RT, &G);
    cvMatMul(&G, mDispToCart, H);
    return status;
}        

#if 0 // TODO: delete them
int Cv3DPoseEstimateDispSpaceRef::checkInLiers(CvMat *uvds0, CvMat *uvds1, CvMat* H){
    return getInLiers(uvds0, uvds1, H, NULL, NULL);
}
int Cv3DPoseEstimateDispSpaceRef::getInLiers(CvMat *uvds0, CvMat *uvds1, CvMat*H, 
    CvMat* uvds0Inlier, CvMat* uvds1Inlier) {
	int numInLiers = 0;
	int numPoints = uvds0->rows;
	double error  = this->mErrThreshold+1;
	
	CvMat* w0 = cvCreateMat(4, 1, CV_64FC1);
	CvMat* w1 = cvCreateMat(4, 1, CV_64FC1);
	for (int i=0; i<numPoints; i++) {
		if (isInLier(uvds0, uvds1, i, H) == true) {
            // store the inlier
            if (uvds0Inlier) {
                cvmSet(uvds0Inlier, numInLiers, 0, cvmGet(uvds0, i, 0));
                cvmSet(uvds0Inlier, numInLiers, 1, cvmGet(uvds0, i, 1));
                cvmSet(uvds0Inlier, numInLiers, 2, cvmGet(uvds0, i, 2));
            }
            if (uvds1Inlier) {
                cvmSet(uvds1Inlier, numInLiers, 0, cvmGet(uvds1, i, 0));
                cvmSet(uvds1Inlier, numInLiers, 1, cvmGet(uvds1, i, 1));
                cvmSet(uvds1Inlier, numInLiers, 2, cvmGet(uvds1, i, 2));
            }
			numInLiers++;
		}
		
#if 0		// TODO: replaced, delete it
		// copy the ith point in uvds0 into a 4x1 matrix of transpose(u, v, d, 1)
		cvSetReal2D(w0, 0, 0, cvmGet(uvds0, i, 0));
		cvSetReal2D(w0, 1, 0, cvmGet(uvds0, i, 1));
		cvSetReal2D(w0, 2, 0, cvmGet(uvds0, i, 2));
		cvSetReal2D(w0, 3, 0, 1.0);
		
		// w1 = H * w0
		cvMatMul(H, w0, w1);
		
		// compare the ith point in uvds1
		double w1_3 = cvmGet(w1, 3, 0);
		
		double error0 = fabs(cvmGet(uvds1, i, 0)-cvmGet(w1, 0, 0)/w1_3);
		double error1 = fabs(cvmGet(uvds1, i, 1)-cvmGet(w1, 1, 0)/w1_3);
		double error2 = fabs(cvmGet(uvds1, i, 2)-cvmGet(w1, 2, 0)/w1_3);
		
		error = (error0 > error1)? error0 : error1;
		error = (error  > error2)? error  : error2;
		
		cout << "i: " << i << " error: " << error << endl;
		
		if (error <= this->mErrThreshold) {
            // store the inlier
            if (uvds0Inlier) {
                cvmSet(uvds0Inlier, numInLiers, 0, cvmGet(uvds0, i, 0));
                cvmSet(uvds0Inlier, numInLiers, 1, cvmGet(uvds0, i, 1));
                cvmSet(uvds0Inlier, numInLiers, 2, cvmGet(uvds0, i, 2));
            }
            if (uvds1Inlier) {
                cvmSet(uvds1Inlier, numInLiers, 0, cvmGet(uvds1, i, 0));
                cvmSet(uvds1Inlier, numInLiers, 1, cvmGet(uvds1, i, 1));
                cvmSet(uvds1Inlier, numInLiers, 2, cvmGet(uvds1, i, 2));
            }
			numInLiers++;
		}
#endif
	}
	
	cout << "Num of Inliers: "<<numInLiers<<endl;
	return numInLiers;
}
#endif

bool Cv3DPoseEstimateDispSpaceRef::setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy){
	Parent::setParams(Fx, Fy, Tx, Clx, Crx, Cy);
	
	// free up the old matrices if they are there
	cvReleaseMat(&mDispToCart);
	cvReleaseMat(&mCartToDisp);

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
	this->mDispToCart = cvCloneMat(&_Q);

	// construct the matrix that maps from cartesian coordinates to 
	// disparity coordinates
	double P[] = 
	{
		mFx, 0., mClx, 0.,
		0., mFy, mCy, 0.,
		0., 0., mClx-mCrx, mFx*mTx,
		0., 0., 1., 0.
	};
	CvMat _P = cvMat(4, 4, CV_64F, P);
	this->mCartToDisp = cvCloneMat(&_P);
	
#if 0 // explicitely computed
	this->mCartToDisp = cvCreateMat(4, 4, CV_64FC1);
	cvInvert(mDispToCart, mCartToDisp);
#endif
#ifdef DEBUG
	// just check if they are invertible of each other
	cout << "Cv3DPoseEstimateDispSpaceRef::setCameraParams()"<<endl;
	CvMat* identMat = cvCreateMat(4, 4, CV_64FC1);
	cvMatMul(this->mCartToDisp, this->mDispToCart, identMat);
	CvMatUtils::printMat(identMat);
	cvReleaseMat(&identMat);
#endif
	return true;	
}

bool Cv3DPoseEstimateDispSpaceRef::reprojection(CvMat *uvds, CvMat *XYZs) {
	bool status = true;
	
	CvMat uvds0;
	CvMat XYZs0;
	cvReshape(uvds, &uvds0, 3, 0);
	cvReshape(XYZs, &XYZs0, 3, 0);
	cvPerspectiveTransform(&uvds0, &XYZs0, this->mDispToCart);
	return status;
}

bool Cv3DPoseEstimateDispSpaceRef::projection(CvMat *XYZs, CvMat *uvds) {
	bool status = true;
	CvMat xyzs0;
	CvMat uvds0;
	cvReshape(XYZs, &xyzs0, 3, 0);
	cvReshape(uvds, &uvds0, 3, 0);
	cvPerspectiveTransform(&xyzs0, &uvds0, this->mCartToDisp);
	return status;
}


