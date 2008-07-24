
#include <iostream>
#include <cxcore.h>
#include "CvTest3DPoseEstimate.h"

#include "Cv3DPoseEstimateDisp.h"
#include "Cv3DPoseEstimate.h"
//#include "Cv3DPoseEstimateRef.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"

using namespace std;

#define GAUSSIANNOISE

CvTest3DPoseEstimate::CvTest3DPoseEstimate():
	Parent(),
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0)
{
	_init();
};
CvTest3DPoseEstimate::CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy), 
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0)
{
	_init();
}

CvTest3DPoseEstimate::~CvTest3DPoseEstimate()
{
}

void CvTest3DPoseEstimate::_init(){
	cvInitMatHeader(&mRot,   3, 3, CV_64FC1, mRotData);
	cvInitMatHeader(&mTrans, 3, 1, CV_64FC1, mTransData);
	
	mTestCartesian = true;
//	mTestCartesian = false;
}

/**
 *  points0  - numPoints x 3
 *  points1  - numPoints x 3
 */
void CvTest3DPoseEstimate::transform(CvMat *points0, CvMat *points1) {
	// create a translated/rotated copy of the 3D points
    // the euler angle of the rotation matrix is given in z-y-x form, aka roll-yaw-pitch, Tait-Bryan angles
	double roll, pitch, yaw;
	pitch = mEulerAngle.x;
	yaw   = mEulerAngle.y;
	roll  = mEulerAngle.z;
	
#if 0 // ZYX
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::ZYX);
	cout << "Rotation Matrix:= Pitch * Yaw* Roll"<< endl;
#else // XYZ euler angle
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::XYZ);
	cout << "Rotation Matrix:= Roll * Yaw * Pitch: "<< mEulerAngle.x/CV_PI*180. << ","<<mEulerAngle.y/CV_PI*180.<<","<<mEulerAngle.z/CV_PI*180.<< endl;
#endif
#if 1
	CvMatUtils::printMat(&mRot);
#endif

	cvmSet(&mTrans, 0, 0, mTranslation.x);
	cvmSet(&mTrans, 1, 0, mTranslation.y);
	cvmSet(&mTrans, 2, 0, mTranslation.z);

#if 1
	cout << "Translation Matrix: "<< endl;
	CvMatUtils::printMat(&mTrans);
#endif
			
	// R * transpose(points0) + T
	transform(&mRot, points0, &mTrans, points1);
}

void CvTest3DPoseEstimate::transform(CvMat* R, CvMat *points0, CvMat* T, CvMat *points1) {
	int numPoints = points0->rows;
	CvMat * points1t =  cvCreateMat(3,  numPoints, CV_64FC1);
	
	CvMat *Temp = cvCreateMat(3, points0->rows, CV_64FC1);
	cvRepeat(T, Temp);
	cvGEMM(R, points0, 1.0, Temp, 1.0, points1t, CV_GEMM_B_T);
	cvTranspose(points1t, points1);
	
#if 0
	cout << "original points: " << endl;
	CvMatUtils::printMat(points0);
	cout << "transformed points: "<<endl;
	CvMatUtils::printMat(points1);
	cout << "Rot and Trans: "<< endl;
	CvMatUtils::printMat(R);
	CvMatUtils::printMat(T);
#endif
}

double CvTest3DPoseEstimate::randReal(double min, double max) {
	double r = cvRandReal(&mRng);
	r = r*(max-min) + min;
	return r;
}

void CvTest3DPoseEstimate::disturb(CvMat *xyzs) {
#ifdef GAUSSIANNOISE
	CvMat* Noise = cvCreateMat(xyzs->rows, xyzs->cols, xyzs->type);
	double sigma = mDisturbScale/2.0; // ~ 95%
	cvRandArr( &mRng, Noise, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
	cvAdd(xyzs, Noise, xyzs);
	cvReleaseMat(&Noise);
#else
	double s = mDisturbScale;
	if (s<=0.0)
		return;
	for (int i=0; i<xyzs->rows; i++){
		cvSetReal2D(xyzs, i, 0, cvGetReal2D(xyzs, i, 0)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 1, cvGetReal2D(xyzs, i, 1)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 2, cvGetReal2D(xyzs, i, 2)+randReal(-s, s));
	}
#endif
}

void CvTest3DPoseEstimate::randomize(CvMat *xyzs, int num, double maxVal){
	// fill in the random number in (-maxVal and maxVal)
	double r;
	for (int i=0; i<num; i++){
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 0, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 1, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 2, r);
	}
}

bool CvTest3DPoseEstimate::test(){
    bool status = true;
	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped_2000.xml");
	
//	CvMat * points0 =  (CvMat *)cvLoad("Data/3dPointClouds0.xml");
//	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped.xml");
	int numPoints = points0->rows;
	double percentageOfOutliers = mOutlierPercentage;
	
	double numOutliers = percentageOfOutliers*numPoints;
	
	CvMat * points1  =  cvCreateMat(numPoints,  3, CV_64FC1);
	
	mEulerAngle.x = CV_PI/4.;
	mEulerAngle.y = CV_PI/3.;
	mEulerAngle.z = CV_PI/6;
	
	mTranslation.x = 100.;
	mTranslation.y = 10.0;
	mTranslation.z = 50.0;
	
#if 0
	mEulerAngle.x = 47.0055/180*CV_PI;
	mEulerAngle.y = -19.7913/180*CV_PI;
	mEulerAngle.z = 30.4901/180*CV_PI;
		
	mTranslation.x = 52.08514;
	   
	mTranslation.y = 106.30646;

	mTranslation.z = 109.97764;
#endif
	
	CvMat *rot    = cvCreateMat(3, 3, CV_64FC1);
	CvMat *trans  = cvCreateMat(3, 1, CV_64FC1);

//	Cv3DPoseEstimateRef peCart;
	Cv3DPoseEstimate peCart;
//	Cv3DPoseEstimateDispSpaceRef peDisp;
	Cv3DPoseEstimateDisp peDisp;
	CvMat *uvds0=NULL, *uvds1=NULL;
	CvScalar mean, std;
	if (this->mTestCartesian == true) {
		cout << "Testing in Cartesian Space"<<endl;
		cvAvgSdv(points0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;
		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 1.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peCart.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;
	} else {
		uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
		uvds1 = cvCreateMat(numPoints, 3, CV_64FC1);
		this->convert3DToDisparitySpace(points0, uvds0);
		cvAvgSdv(uvds0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

//		this->mDisturbScale = 1.5;
//		this->mOutlierScale = 10.0;
		this->mDisturbScale = 1.0;
		this->mOutlierScale = 10.0;
		mOutlierPercentage = 0.2;
		// set threshold
		double threshold = 1.0;
		peCart.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;
		
		peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);
	}
	
	double numIters = 1000;
	double maxErrorAfterLevMarq = 0.0;
	int numInliers_maxErrorAfterLevMarq=0;
	double maxErrorBeforeLevMarq = 0.0;
	int numInliers_maxErrorBeforeLevMarq=0;
	int maxNumInliers = 0;
	double errAfterLevMarq_maxNumInliers=0.0;
	double maxImprovementAfterLevMarq = 0;
	int numInliers_maxImprovementAfterLevMarq;
	
	int numGoodIters=0;
	for (int i=0; i<numIters; i++)
	{
		cout << "Test Case Number:  "<<i<<endl;
#if 1
		mEulerAngle.x = CV_PI*randReal(-1., 1.);
		mEulerAngle.y = CV_PI*randReal(-1., 1.);
		mEulerAngle.z = CV_PI*randReal(-1., 1.);

		mTranslation.x = 50. + 100.*cvRandReal(&mRng);
		mTranslation.y = 50. + 100.*cvRandReal(&mRng);
		mTranslation.z = 50. + 100.*cvRandReal(&mRng);
#endif
		transform(points0, points1);
		
		// now randomize the first percentageOfOutliers points to make them outliers
		randomize(points1, numOutliers, mOutlierScale);

		int numInLiers;
		CvMat *inliers0=NULL, *inliers1=NULL;

//		double _data[16];
//		double _data2[16];
//		CvMat  TransformBestBeforeLevMarq = cvMat(4, 4, CV_64F, _data);
//		CvMat  TransformAfterLevMarq = cvMat(4, 4, CV_64F, _data2);
		CvMat *TransformBestBeforeLevMarq;
		CvMat *TransformAfterLevMarq;
		if (this->mTestCartesian) {
			disturb(points1);
			int64 t = cvGetTickCount();
			numInLiers = peCart.estimate(points0, points1, rot, trans, inliers0, inliers1);
			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
//			cvCopy(&peCart.mRTBestWithoutLevMarq, &TransformBestBeforeLevMarq);
			TransformBestBeforeLevMarq = peCart.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peCart.getFinalTransformation();
		} else {
			// convert both set of points into disparity color space
			this->convert3DToDisparitySpace(points1, uvds1);
			
			disturb(uvds1);
			
			int64 t = cvGetTickCount();
			numInLiers = peDisp.estimate(uvds0, uvds1, rot, trans, inliers0, inliers1);	
			
			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
//			cvCopy(&peDisp.mRTBestWithoutLevMarq, &TransformBestBeforeLevMarq);
			TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peDisp.getFinalTransformation();
		}
		if (numInLiers < 6) {
			continue;
		}
		numGoodIters++;

#if 1
		cout << "Max num of InLiers: "<< numInLiers << endl;
		cout << "Reconstructed Rotation Matrix" << endl;
		CvMatUtils::printMat(rot);
		cout << "Reconstructed Translation Matrix" << endl;
		CvMatUtils::printMat(trans);

		CvMat *points1r = cvCreateMat(numPoints, 3, CV_64FC1);
		this->transform(rot, points0, trans, points1r);
		//	cout << "Reconstructed tranformed points: "<<endl;
		//	CvMatUtils::printMat(points1r);

		// calculate the relative L2 norm of the diff between true transformation and predicted transformation
		double errRotMat   = cvNorm(rot, &mRot, CV_L2)/3.0;
		double errTransMat = cvNorm(trans, &mTrans, CV_RELATIVE_L2);

		cout << "L2 -norm of the diff of rot mat and trans mat are: "<<errRotMat<<" , "<<errTransMat<<endl;

		if (inliers0){
			CvMat *inliers1r = cvCreateMat(numInLiers, 3, CV_64FC1);
			
			CvMat P0, P1r;
			
			cvReshape(inliers0, &P0, 3, 0);
			cvReshape(inliers1r, &P1r, 3, 0);
			
			cvPerspectiveTransform(&P0, &P1r, TransformBestBeforeLevMarq);
			double errorInliersBefore = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (Before LevMarq):  "<< errorInliersBefore << endl;
			if (maxErrorBeforeLevMarq < errorInliersBefore) {
				maxErrorBeforeLevMarq = errorInliersBefore;
				numInliers_maxErrorBeforeLevMarq = numInLiers;
			}

			cvPerspectiveTransform(&P0, &P1r, TransformAfterLevMarq);
//			this->transform(rot, inliers0, trans, inliers1r);
			//	cout << "Reconstructed tranformed points of inliers: "<<endl;
			// calculate the relative L2 norm of the diff between observed and predicted points;
			double errorInliersAfter = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (After  LevMarq):  "<< errorInliersAfter << endl;
			if (maxErrorAfterLevMarq < errorInliersAfter) {
				maxErrorAfterLevMarq = errorInliersAfter;
				numInliers_maxErrorAfterLevMarq = numInLiers;
			}
			
			if (maxNumInliers < numInLiers){
				maxNumInliers = numInLiers;
				errAfterLevMarq_maxNumInliers = errorInliersAfter;
			}
			
			if (maxImprovementAfterLevMarq < errorInliersBefore - errorInliersAfter){
				maxImprovementAfterLevMarq = errorInliersBefore - errorInliersAfter;
				numInliers_maxImprovementAfterLevMarq = numInLiers;
			}


			cvReleaseMat(&inliers1r);
		}

		// calculate the relative L2 norm of the diff between observed and predicted points;
		double error = cvNorm(points1, points1r, CV_RELATIVE_L2)/numPoints;
		cout << "Average of the L2-norm of the diff between observed and prediction:  "<< error << endl;

		cvReleaseMat(&points1r);
#endif
	}
	cout << "max error before levmarq: "<< maxErrorBeforeLevMarq << endl;
	cout << "num of inliers for the test case: "<< numInliers_maxErrorBeforeLevMarq << endl;
	
	cout << "max error after levmarq:  " << maxErrorAfterLevMarq  << endl;
	cout << "num of inliers for the test case: " << numInliers_maxErrorAfterLevMarq<< endl;
	
	cout << "max improvement after levmarq: " << maxImprovementAfterLevMarq << endl;
	cout << "num of inliers for the test case: "<<numInliers_maxImprovementAfterLevMarq << endl;
	
	cout << "max num of inliers: "<< maxNumInliers << endl;
	cout << "error after levmarq for the test case: "<< errAfterLevMarq_maxNumInliers << endl; 

	CvTestTimer& timer = CvTestTimer::getTimer();
	
	timer.mNumIters = numGoodIters;
	timer.printStat();
	
	
    return status;
}

int main(){
	cout << "Testing wg3DPoseEstimate ..."<<endl;
	
	
	CvTest3DPoseEstimate test3DPoseEstimate;
    
    test3DPoseEstimate.test();
	
	cout << "Done testing wg3DPoseEstimate ..."<<endl;
	return 0;
}
