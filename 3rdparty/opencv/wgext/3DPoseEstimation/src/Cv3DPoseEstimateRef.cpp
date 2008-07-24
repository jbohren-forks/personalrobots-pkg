#include "iostream.h"

#include "cv.h"

#include "CvMat3X3.h"
#include "CvLevMarqDispSpace.h"
#include "Cv3DPoseEstimateRef.h"
#include "CvMatUtils.h"
#include "CvTestTimer.h"

//#define DEBUG
//#define USE_LEVMARQ

Cv3DPoseEstimateRef::Cv3DPoseEstimateRef():
	mNumIterations(20), mMinDet(0.1), mNumTriesForRandomTriple(10), 
	mErrMapping(NULL), mErrNormType(CV_C),	mErrThreshold(mDefErrThreshold),
	mResidue1(cvMat(3, 1, CV_64F, mResidue1_Data)),
	mResidue2(cvMat(3, 1, CV_64F, mResidue2_Data)),
	mW1(cvMat(4, 1, CV_64F, mW1_Data)),
	mT(cvMat(4, 4, CV_XF, mT_Data)),
	mRng(time(NULL))
{
	mRTBestWithoutLevMarq = cvMat(4, 4, CV_XF, mRTBestWithoutLevMarqData);
}

Cv3DPoseEstimateRef::~Cv3DPoseEstimateRef()
{
}

void Cv3DPoseEstimateRef::configureErrorMeasurement(CvMat *mapping, double threshold, int normType) {
	this->mErrMapping   = mapping;
	this->mErrThreshold = threshold;
	this->mErrNormType  = normType;
}

bool Cv3DPoseEstimateRef::estimateLeastSquare(CvMat *p0, CvMat *p1, CvMat *R, CvMat *T) {
	CvMat *P0 = cvCreateMat(p0->cols, p0->rows, CV_64FC1);
	CvMat *P1 = cvCreateMat(p1->cols, p1->rows, CV_64FC1);
	cvTranspose(p0, P0);
	cvTranspose(p1, P1);
	
	return estimateLeastSquareInCol(P0, P1, R, T);
}

bool Cv3DPoseEstimateRef::estimateLeastSquareInCol(CvMat *P0, CvMat *P1, CvMat *R, CvMat *T) {
	bool status = true;
	double _Q[9], _W[9], _Ut[9], _Vt[9], _C0[3], _C1[3];
	CvMat Q  = cvMat(3, 3, CV_64F, _Q); // Q = P1 * transpose(P0)
	CvMat W  = cvMat(3, 3, CV_64F, _W); // Eigen matrix of Q. Q = U * W * transpose(V)
	CvMat Ut = cvMat(3, 3, CV_64F, _Ut); // transpose(U)
	CvMat Vt = cvMat(3, 3, CV_64F, _Vt); // transpose(V)
	CvMat C0 = cvMat(3, 1, CV_64F, _C0); // centroid of the 3 points in P0
	CvMat C1 = cvMat(3, 1, CV_64F, _C1); // centroid of the 3 points in P1
	// compute the centroids of these 3 ponints for the two positions
	//cvCVAPI(void)  cvReduce( const CvArr* src, CvArr* dst, int dim CV_DEFAULT(-1),
    //               int op CV_DEFAULT(CV_REDUCE_SUM) );
	cvReduce(P0, &C0, -1, CV_REDUCE_AVG);
	// compute the relative vectors of the two groups of points w.r.t. their centroids.
	double _Temp[3*P0->cols];
	CvMat Temp = cvMat(3, P0->cols, CV_64F, _Temp);
	cvRepeat(&C0, &Temp);
	cvSub(P0, &Temp, P0);

	cvReduce(P1, &C1, -1, CV_REDUCE_AVG);
	cvRepeat(&C1, &Temp);
	cvSub(P1, &Temp, P1);
	
	// Q = P1 * P0^T
	cvGEMM(P1, P0, 1, NULL, 0, &Q, CV_GEMM_B_T);
	
	// do a SVD on Q. Q = U*W*transpose(V)
	// according to the documentation, specifying the flags speeds up the processing
	cvSVD(&Q, &W, &Ut, &Vt, CV_SVD_MODIFY_A|CV_SVD_U_T|CV_SVD_V_T);
	
	// R = U * S * V^T
	double _S[] = {
			1.,0.,0.,
			0.,1.,0.,
			0.,0.,1.
	};
	CvMat S;
	cvInitMatHeader(&S, 3, 3, CV_64FC1, _S);
	if (cvDet(&Ut)*cvDet(&Vt)<0) {
		// Ut*V is not a rotation matrix, although either of them are unitary
		// we shall set the last diag entry of S to be -1
		// so that Ut*S*V is a rotational matrix
		// setting S[2,2] = -1
		_S[8] = -1.;
		double _UxS[9];
		CvMat UxS;
		cvInitMatHeader(&UxS, 3, 3, CV_64FC1, _UxS);
		cvGEMM(&Ut, &S, 1, NULL, 0, &UxS, CV_GEMM_A_T);
		cvGEMM(&UxS, &Vt, 1, NULL, 0, R, 0);
	} else {
		cvGEMM(&Ut, &Vt, 1, NULL, 0, R, CV_GEMM_A_T);
	}

	// t = centroid1 - R*centroid0
	cvGEMM(R, &C0, -1, &C1, 1, T, 0);
	return status;
}

// TODO: this function is now not consistency between RT and mT
int Cv3DPoseEstimateRef::estimate(CvMat *points0, CvMat *points1, CvMat *rot, CvMat *trans,
		CvMat *&inliers0, CvMat *&inliers1){
	int numInLiers = 0;
	
	int numPoints = points0->rows;
	
	if (numPoints != points1->rows) {
		cerr << "number of points mismatched in input" << endl;
		return 0;
	}
	
	double _P0[3*3], _P1[3*3], _R[3*3], _T[3*1], _RT[4*4];
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
		
		CvTestTimerStart(SVD);
		this->estimateLeastSquareInCol(&P0, &P1, &R, &T);
		CvTestTimerEnd(SVD);
        
//        this->constructRT(&R, &T, &RT);
        this->constructRT(&R, &T, &mT);
		
        CvTestTimerStart(CheckInliers)
		// scoring against all points
		numInLiers = checkInLiers(points0, points1, &RT);
        CvTestTimerEnd(CheckInliers)
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
    
	// get a copy of all the inliers, original and transformed
    CvMat *points0Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    CvMat *points1Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
    // construct homography matrix
    constructRT(rot, trans, &RT);
    int numInLiers0 = getInLiers(points0, points1, &RT, points0Inlier, points1Inlier);
    
    cout << "Number of Inliers: "<< numInLiers0 << endl;
    if (numInLiers0<6) {
    	cout << "Too few inliers: "<< numInLiers0 << endl;
    	return numInLiers0;
    }
	
#ifdef USE_LEVMARQ    
	
	// get the euler angle from rot
	CvPoint3D64f eulerAngles;
	{
		double _R[9], _Q[9];
        CvMat R, Q;
        CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
        cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
        cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

#ifdef DEBUG // all debugging stuff      
//#if 1
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
//#if 1
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
    CvLevMarq3D levMarq(numInLiers0);
    
    double param[6];
    
    //initialize the parameters
    param[0] = eulerAngles.x/180. * CV_PI;
    param[1] = eulerAngles.y/180. * CV_PI;
    param[2] = eulerAngles.z/180. * CV_PI;
    param[3] = cvmGet(trans, 0, 0);
    param[4] = cvmGet(trans, 1, 0);
    param[5] = cvmGet(trans, 2, 0);

    printf("initial parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
    		param[3], param[4], param[5]);
    
#if 0
    for (int i=0; i<6; i++) param[i] *= 1.2;
    printf("disturbed parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
    		param[3], param[4], param[5]);
#endif
    levMarq.doit(points0Inlier, points1Inlier, param);    

#ifdef DEBUG
    printf("optimized parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
    		param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2], 
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
    
#endif    
    inliers0 = points0Inlier;
    inliers1 = points1Inlier;
	return numInLiers0;	
}

bool Cv3DPoseEstimateRef::colinear(CvMat* points){
	CvMat * temp = cvCloneMat(points);
	cvSetReal2D(temp, 0, 0, 1);
	cvSetReal2D(temp, 0, 1, 1);
	cvSetReal2D(temp, 0, 2, 1);
	if (cvDet(temp)>this->mMinDet){
		return false;
	}
	cvCopy(points, temp);
	cvSetReal2D(temp, 1, 0, 1);
	cvSetReal2D(temp, 1, 1, 1);
	cvSetReal2D(temp, 1, 2, 1);
	if (cvDet(temp)>this->mMinDet){
		return false;
	}
	cvCopy(points, temp);
	cvSetReal2D(temp, 2, 0, 1);
	cvSetReal2D(temp, 2, 1, 1);
	cvSetReal2D(temp, 2, 2, 1);
	if (cvDet(temp)>this->mMinDet){
		return false;
	}
	// TODO: release temp
	return true;
}

bool Cv3DPoseEstimateRef::pick3RandomPoints(CvMat* points0, CvMat* points1, CvMat* P0, CvMat* P1, 
		bool fInputPointsInRows){
	bool status = false;
	int numPoints = points0->rows;
	
	for (int i=0; i<mNumTriesForRandomTriple; i++){
		
		int pa = cvRandInt(&mRng) % numPoints;
		int pb, pc;
		do { pb = cvRandInt(&mRng) % numPoints; } while ( pb == pa);
		do { pc = cvRandInt(&mRng) % numPoints; } while ( pc == pa || pc == pb);
		
		// points in P0, and P1 are stored in columns
		if (fInputPointsInRows == true ) {
			cvSetReal2D(P0, 0, 0, cvmGet(points0, pa, 0));
			cvSetReal2D(P0, 1, 0, cvmGet(points0, pa, 1));
			cvSetReal2D(P0, 2, 0, cvmGet(points0, pa, 2));

			cvSetReal2D(P1, 0, 0, cvmGet(points1, pa, 0));
			cvSetReal2D(P1, 1, 0, cvmGet(points1, pa, 1));
			cvSetReal2D(P1, 2, 0, cvmGet(points1, pa, 2));

			cvSetReal2D(P0, 0, 1, cvmGet(points0, pb, 0));
			cvSetReal2D(P0, 1, 1, cvmGet(points0, pb, 1));
			cvSetReal2D(P0, 2, 1, cvmGet(points0, pb, 2));

			cvSetReal2D(P1, 0, 1, cvmGet(points1, pb, 0));
			cvSetReal2D(P1, 1, 1, cvmGet(points1, pb, 1));
			cvSetReal2D(P1, 2, 1, cvmGet(points1, pb, 2));

			cvSetReal2D(P0, 0, 2, cvmGet(points0, pc, 0));
			cvSetReal2D(P0, 1, 2, cvmGet(points0, pc, 1));
			cvSetReal2D(P0, 2, 2, cvmGet(points0, pc, 2));

			cvSetReal2D(P1, 0, 2, cvmGet(points1, pc, 0));
			cvSetReal2D(P1, 1, 2, cvmGet(points1, pc, 1));
			cvSetReal2D(P1, 2, 2, cvmGet(points1, pc, 2));
		} else {
			cerr << "Not Implemented Yet"<<endl;
			status = false;
			return status;
		}
		
#ifdef DEBUG
		printf("random points: %d, %d, %d\n", pa, pb, pc);
		cout << "P0:"<<endl;
		CvMatUtils::printMat(P0);
		cout << "P1:"<<endl;
		CvMatUtils::printMat(P1);
		CvMatUtils::printMat(points1);
#endif
		
		// TODO: check if they are colinear
		if (colinear(P0) == false && colinear(P1) == false) {
			status = true;
#ifdef DEBUG
			cout << "det(P0) = "<< cvDet(P0) << "  "<< "det(P1) = " << cvDet(P1) << endl;
#endif
			break;
		}
	}
	return status;
}

bool Cv3DPoseEstimateRef::constructRT(CvMat *R, CvMat *T, CvMat *RT){
	bool status = true;
    // construct RT
    for (int r=0; r<3; r++) {
        for (int c=0; c<3; c++) {
            cvSetReal2D(RT, r, c, cvmGet(R, r, c));
        }
        cvSetReal2D(RT, r, 3, cvmGet(T, r, 0));
    }
    
    cvSetReal2D(RT, 3, 0, 0.0);
    cvSetReal2D(RT, 3, 1, 0.0);
    cvSetReal2D(RT, 3, 2, 0.0);
    cvSetReal2D(RT, 3, 3, 1.0);
	return status;
}

int Cv3DPoseEstimateRef::checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation){
    return getInLiers(points0, points1, transformation, NULL, NULL);
}

bool Cv3DPoseEstimateRef::isInLier(CvMat *points0, CvMat *points1, int i){
	{
		CvMyReal p0x, p0y, p0z;

//		int64 t01 = cvGetTickCount();
		
		p0x = cvmGet(points0, i, 0);
		p0y = cvmGet(points0, i, 1);
		p0z = cvmGet(points0, i, 2);
		
		CvMyReal w0 = mT_Data[ 0]*p0x + mT_Data[ 1]*p0y + mT_Data[ 2]*p0z + mT_Data[ 3];
		CvMyReal w1 = mT_Data[ 4]*p0x + mT_Data[ 5]*p0y + mT_Data[ 6]*p0z + mT_Data[ 7];
		CvMyReal w2 = mT_Data[ 8]*p0x + mT_Data[ 9]*p0y + mT_Data[10]*p0z + mT_Data[11];
		CvMyReal w3 = mT_Data[12]*p0x + mT_Data[13]*p0y + mT_Data[14]*p0z + mT_Data[15];
			
//		int64 t02 = cvGetTickCount();
			
		CvMyReal scale;

		mResidue1_Data[0] = cvmGet(points1, i, 0)-w0*(scale=1./w3);
		mResidue1_Data[1] = cvmGet(points1, i, 1)-w1*scale;
		mResidue1_Data[2] = cvmGet(points1, i, 2)-w2*scale;
	}
	
	// computing the norm
	double error;
	if (mErrMapping == NULL){
		error = cvNorm(&mResidue1, NULL, this->mErrNormType);
	} else {
		cout << "Not Implemented Yet"<<endl;
		cvMatMul(mErrMapping, &mResidue1, &mResidue2);
		error = cvNorm(&mResidue2, NULL, this->mErrNormType);
	}
	
#ifdef DEBUG			
	cout << "i: " << i << " error: " << error << endl;
#endif
	return error <= this->mErrThreshold;
}

int Cv3DPoseEstimateRef::getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
    CvMat* points0Inlier, CvMat* points1Inlier) {
	
	int numInLiers = 0;
	int numPoints = points0->rows;
	
	for (int i=0; i<numPoints; i++) {
		
		if (isInLier(points0, points1, i) == true) {
            // store the inlier
            if (points0Inlier) {
                cvmSet(points0Inlier, numInLiers, 0, cvmGet(points0, i, 0));
                cvmSet(points0Inlier, numInLiers, 1, cvmGet(points0, i, 1));
                cvmSet(points0Inlier, numInLiers, 2, cvmGet(points0, i, 2));
            }
            if (points1Inlier) {
                cvmSet(points1Inlier, numInLiers, 0, cvmGet(points1, i, 0));
                cvmSet(points1Inlier, numInLiers, 1, cvmGet(points1, i, 1));
                cvmSet(points1Inlier, numInLiers, 2, cvmGet(points1, i, 2));
            }
			numInLiers++;
		}
	}
#ifdef DEBUG	
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;
}

