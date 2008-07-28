#include <cv.h>

#include "CvMat3X3.h"
#include "Cv3DPoseEstimate.h"
#include "CvLevMarq3D.h"

#include <iostream>
#include "CvTestTimer.h"
using namespace std;

//#define DEBUG
#define USE_LEVMARQ
//#define DEBUG_DISTURB_PARAM

#if 1
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

Cv3DPoseEstimate::Cv3DPoseEstimate()
{
}

Cv3DPoseEstimate::~Cv3DPoseEstimate()
{
}

int Cv3DPoseEstimate::checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation){
	int numInLiers = 0;
	int numPoints = points0->rows;
	
	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows, 
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;  
	for (int i=0; i<numPoints; i++) {
		
		CvMyReal p0x, p0y, p0z;

#if 1
		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
#else 
		// not worth using the following code
		// 1) not sure if it helps on speed. 
		// 2) not sure if the order of evaluation is preserved as left to right.
		CvMyReal w3 = 
			_T[12]*(p0x=*_P0++) + 
			_T[13]*(p0y=*_P0++) + 
			_T[14]*(p0z=*_P0++) + 
			_T[15];
#endif

		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

		numInLiers++;
	}
#ifdef DEBUG	
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;	
}

// almost the same as the function above
int Cv3DPoseEstimate::getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
    CvMat* points0Inlier, CvMat* points1Inlier) {
	int numInLiers = 0;
	int numPoints = points0->rows;
	
	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows, 
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;  
	for (int i=0; i<numPoints; i++) {
		
		CvMyReal p0x, p0y, p0z;

		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

        // store the inlier
        if (points0Inlier) {
            cvmSet(points0Inlier, numInLiers, 0, p0x);
            cvmSet(points0Inlier, numInLiers, 1, p0y);
            cvmSet(points0Inlier, numInLiers, 2, p0z);
        }
        if (points1Inlier) {
            cvmSet(points1Inlier, numInLiers, 0, *(_P1-3));
            cvmSet(points1Inlier, numInLiers, 1, *(_P1-2));
            cvmSet(points1Inlier, numInLiers, 2, *(_P1-1));
        }
		numInLiers++;
	}
#ifdef DEBUG	
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;	
}

int Cv3DPoseEstimate::estimate(CvMat *points0, CvMat *points1, CvMat *rot, CvMat *trans,
		CvMat *&inliers0, CvMat *&inliers1){
	int numInLiers = 0;
	
	int numPoints = points0->rows;
	
	if (numPoints != points1->rows) {
		cerr << "number of points mismatched in input" << endl;
		return 0;
	}
	
	double _P0[3*3], _P1[3*3], _R[3*3], _T[3*1];
	double _RT[16];
	CvMat P0, P1;
	cvInitMatHeader(&P0, 3, 3, CV_64FC1, _P0); // 3 random points from camera0, stored in columns
	cvInitMatHeader(&P1, 3, 3, CV_64FC1, _P1); // 3 random points from caemra1, stored in columns

	CvMat R, T, RT;
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);  // rotation matrix, R = V * tranpose(U)
	cvInitMatHeader(&T,  3, 1, CV_64FC1, _T);  // translation matrix
	cvInitMatHeader(&RT, 4, 4, CV_64FC1, _RT); // transformation matrix (including rotation and translation)
	
	int maxNumInLiers=0;
	for (int i=0; i< mNumIterations; i++) {
#ifdef DEBUG
		cout << "Iteration: "<< i << endl;
#endif
		// randomly pick 3 points. make sure they are not 
		// colinear
		pick3RandomPoints(points0, points1, &P0, &P1);
		
		TIMERSTART2(SVD);
		this->estimateLeastSquareInCol(&P0, &P1, &R, &T);
		TIMEREND2(SVD);
        
        this->constructRT(&R, &T, &RT);
		
        CvTestTimerStart(CheckInliers);
		// scoring against all points
		numInLiers = checkInLiers(points0, points1, &RT);
		CvTestTimerEnd(CheckInliers);
#ifdef DEBUG		
		cout << "R, T, and RT: "<<endl;
		CvMatUtils::printMat(&R);
		CvMatUtils::printMat(&T);
		CvMatUtils::printMat(&RT);
#endif		
		
		// keep the best R and T
		if (maxNumInLiers < numInLiers) {
			maxNumInLiers = numInLiers;
			cvCopy(&R, rot);
			cvCopy(&T, trans);
		}
	}
	
	if (maxNumInLiers<6) {
		cout << "Too few inliers: "<< maxNumInLiers << endl;
		return maxNumInLiers;
	}
    
	int numInLiers0;
    CvMat *points0Inlier;
    CvMat *points1Inlier;
	TIMERSTART(CopyInliers);
	// get a copy of all the inliers, original and transformed
    points0Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    points1Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    // construct homography matrix
    constructRT(rot, trans, &RT);
    
    numInLiers0 = getInLiers(points0, points1, &RT, points0Inlier, points1Inlier);
    
    if (numInLiers0 == 0) {
    	cout << "ZeroInLiers"<<endl;
    	numInLiers0 = getInLiers(points0, points1, &RT, points0Inlier, points1Inlier);
    }
    TIMEREND(CopyInliers);
    
    // make a copy of the best RT before nonlinear optimization
    cvCopy(&RT, &mRTBestWithoutLevMarq);
	
#ifdef USE_LEVMARQ    
    TIMERSTART(LevMarq);
	// get the euler angle from rot
	CvPoint3D64f eulerAngles;
	{
		double _R[9], _Q[9];
        CvMat R, Q;
        CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
        cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
        cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

        cvRQDecomp3x3((const CvMat*)rot, &R, &Q, pQx, pQy, pQz, &eulerAngles);
        
	}

    // nonlinear optimization by Levenberg-Marquardt
    CvLevMarq3D levMarq(numInLiers0);
    
    double param[6];
    
    //initialize the parameters
    param[0] = eulerAngles.x/180. * CV_PI;
    param[1] = eulerAngles.y/180. * CV_PI;
    param[2] = eulerAngles.z/180. * CV_PI;
    param[3] = cvmGet(trans, 0, 0);
    param[4] = cvmGet(trans, 1, 0);
    param[5] = cvmGet(trans, 2, 0);

#ifdef DEBUG
//#if 1
    printf("initial parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0], param[0]/CV_PI*180., param[1], param[1]/CV_PI*180., 
    		param[2], param[2]/CV_PI*180., 
    		param[3], param[4], param[5]);
#endif
#ifdef DEBUG_DISTURB_PARAM
    for (int i=0; i<6; i++) param[i] *= 1.1;
#endif
#ifdef DEBUG
//#if 1
    printf("disturbed parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0], param[0]/CV_PI*180., param[1], param[1]/CV_PI*180., 
    		param[2], param[2]/CV_PI*180., 
    		param[3], param[4], param[5]);
#endif
    int64 tDoit = cvGetTickCount();
    levMarq.doit(points0Inlier, points1Inlier, param);   
    CvTestTimer::getTimer().mLevMarqDoit += cvGetTickCount() - tDoit;

#ifdef DEBUG
//#if 1
    printf("optimized parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0], param[0]/CV_PI*180., param[1], param[1]/CV_PI*180., 
    		param[2], param[2]/CV_PI*180., 
    		param[3], param[4], param[5]);
#endif
    
	// TODO: construct matrix with parameters from nonlinear optimization
    double _rot[9];
    
    CvMat3X3<double>::rotMatrix(param[0], param[1], param[2], _rot, CvMat3X3<double>::XYZ);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) {
            cvmSet(rot, i, j, _rot[i*3+j]);
        }
    cvmSet(trans, 0, 0, param[3]);
    cvmSet(trans, 1, 0, param[4]);
    cvmSet(trans, 2, 0, param[5]);
    
    TIMEREND(LevMarq);
#endif    
    inliers0 = points0Inlier;
    inliers1 = points1Inlier;
    
    this->constructRT(rot, trans, &mT);
	return maxNumInLiers;	
}




