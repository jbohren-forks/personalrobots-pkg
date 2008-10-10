#include <cxcore.h>
#include "LevMarqTransformDispSpace.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
using namespace cv::willow;

#include <iostream>
using namespace std;

#undef DEBUG

#define USE_UPDATEALT
#define LAST3ISLIN

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

LevMarqTransformDispSpace::LevMarqTransformDispSpace(
            const CvMat* disparityTo3D, const CvMat *threeDToDisparity,
            int numMaxIter, AngleType angleType):
	LevMarqTransform(
			numMaxIter,
			angleType
	), m3DToDisparity(threeDToDisparity), mDisparityTo3D(disparityTo3D)
{
//  mAngleType = Euler;
	cvInitMatHeader(&mHomography, 4, 4, CV_64FC1, _Homography);
}

LevMarqTransformDispSpace::~LevMarqTransformDispSpace()
{
}


bool LevMarqTransformDispSpace::constructHomographyMatrix(const CvMat* param){
	return constructHomographyMatrix(param, _Homography);
}

bool LevMarqTransformDispSpace::constructHomographyMatrix(const CvMat* param, double _H[]){
  double _buf[16], _rt[16];
  CvMat buf, rt;
  CvMat H;
  cvInitMatHeader(&buf, 4, 4, CV_64FC1, _buf);
  cvInitMatHeader(&rt, 4, 4, CV_64FC1, _rt);
  cvInitMatHeader(&H, 4, 4, CV_64FC1, _H);
	bool status = true;
	status = constructRTMatrix(param, _rt);
	_rt[15] = 1.0;
	_rt[12] = _rt[13] = _rt[14] = 0.0;

	cvMatMul(m3DToDisparity, &rt, &buf);
	cvMatMul(&buf, mDisparityTo3D, &H);
#if DEBUG
	cout << "cvLevMarqDispSpace::constructHomographyMatrix"<<endl;
	CvMatUtils::printMat(&H);
	cout << "="<<endl;
	CvMatUtils::printMat(m3DToDisparity);
	CvMatUtils::printMat(&rt);
	CvMatUtils::printMat(mDisparityTo3D);
#endif
	return status;
}

bool LevMarqTransformDispSpace::constructTransformationMatrix(const CvMat * param){
	return constructHomographyMatrix(param);
}

bool LevMarqTransformDispSpace::constructTransformationMatrix(const CvMat * param, double T[]) {
	return constructHomographyMatrix(param, T);
}

bool LevMarqTransformDispSpace::constructTransformationMatrices(const CvMat *param, double delta) {

	constructTransformationMatrix(param);

	double _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64F, _param1);
	// transformation matrices for each parameter
	for (int k=0; k<numParams; k++) {
		cvCopy(param, &param1);
		_param1[k] += delta;
		constructTransformationMatrix(&param1, mFwdTData[k]);
	}
	return true;
}


bool LevMarqTransformDispSpace::computeResidueVector(const CvMat *uvdws0, const CvMat *uvdws1,
		CvMat * resVector){
	int numPoints = uvdws0->rows;
	CvMat uvdw0;
	CvMat uvdw1r;
	CvMat uvdw1;
	CvMat uvdw2;
	CvMat uvdw1T;
	double _uvdw1r[4];
	cvInitMatHeader(&uvdw1r, 4, 1, CV_64FC1, _uvdw1r);

	for (int i=0; i<numPoints; i++) {
		cvGetRow(uvdws0, &uvdw0, i);
		cvGetRow(uvdws1, &uvdw1, i);
		cvGetCol(resVector, &uvdw2, i);
		cvGEMM(&mHomography, &uvdw0, 1.0, NULL, -1.0, &uvdw1r, CV_GEMM_B_T);
		double scale = 1.0/cvmGet(&uvdw1r, 3, 0);
		cvScale(&uvdw1r, &uvdw1r, scale);
		cvReshape(&uvdw1, &uvdw1T, 0, 4);
		cvSub(&uvdw1r, &uvdw1T, &uvdw2);
	}
#if DEBUG
	cout << "residue" << endl;
	CvMatUtils::printMat(resVector);
#endif
	return true;
}

bool LevMarqTransformDispSpace::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1, CvMat* res){
	return computeResidue(xyzs0, xyzs1, &mHomography, res);
}

bool LevMarqTransformDispSpace::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1,
		const CvMat *T, CvMat* res){
	TIMERSTART2(Residue);
	CvMat _xyzs0;
	CvMat _res;
	cvReshape(xyzs0, &_xyzs0, 3, 0);
	cvReshape(res, &_res, 3, 0);
	cvPerspectiveTransform(&_xyzs0, &_res, T);
	cvSub(res, xyzs1, res);
#if DEBUG
	cout << "residue" << endl;
	CvMatUtils::printMat(res);
#endif
	TIMEREND2(Residue);
	return true;
}

// This function shall be the same as LevMarqTransform::optimizeAlt except for the
// residue computation
bool LevMarqTransformDispSpace::optimizeAlt(const CvMat *xyzs0, const CvMat *xyzs1, double _param[]){
	bool status=true;
	//initialize the initial vector of parameters
	if (_param == NULL){
		cvSetZero(mLevMarq.param);
	} else {
		for (int i=0; i<numParams; i++) {
			cvSetReal2D(mLevMarq.param, i, 0, _param[i]);
		}
	}

	int numPoints = xyzs0->rows;
	if (numPoints != xyzs1->rows) {
		cerr << "Fatal Error, num of points unmatched in input"<<endl;
	}

	double delta = CV_PI/(180.*100.);

	double _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64FC1, _param1);

	double _r0[3];
	CvMat r0 = cvMat(1, 3, CV_64FC1, _r0);
	double _r1[3*numParams];
	CvMat r1 = cvMat(numParams, 3, CV_64FC1, _r1);

	int iterUpdates;
	for(iterUpdates=0; iterUpdates<=defMaxTimesOfUpdates	;	iterUpdates++	) {
		const CvMat *param0=NULL;
		CvMat *_JtJ=NULL;
		CvMat * _JtErr=NULL;
		double *_errNorm=NULL;
		bool moreUpdate;
		TIMERSTART(LevMarq)
		moreUpdate = mLevMarq.updateAlt(param0,
				_JtJ, _JtErr, _errNorm );
		TIMEREND(LevMarq)
		if (moreUpdate == false) {
			break;
		}

		if( _JtJ )
			cvZero( _JtJ );
		if( _JtErr )
			cvZero( _JtErr );
		if( _errNorm )
			*_errNorm = 0;

		if( _JtJ || _JtErr )
		{
			double scale = 1./delta;

			// Not sure if this is illegal;
			double* JtJData   = _JtJ->data.db;
			double* JtErrData = _JtErr->data.db;

			constructTransformationMatrices(param0, delta);

			// loop thru all the data points. At each iteration
			// update each entry of JtJ with the contribution from the error (residue)
			// from the data point.
			for (int j=0; j<numPoints; j++) {
				// compute current error = xyzs1^T  - Transformation * xyzs0^T
				CvMat point0, point1;
				cvGetRow(xyzs0, &point0, j);
				cvGetRow(xyzs1, &point1, j);
				double _r0x;
				double _r0y;
				double _r0z;

				//	        	xyzs0 and xyzs1's are inliers we copy. so we know
				//	        	how their data are organized
				double _p0x = cvmGet(xyzs0, j, 0);
				double _p0y = cvmGet(xyzs0, j, 1);
				double _p0z = cvmGet(xyzs0, j, 2);

				double _p1x = cvmGet(xyzs1, j, 0);
				double _p1y = cvmGet(xyzs1, j, 1);
				double _p1z = cvmGet(xyzs1, j, 2);
				PERSTRANSFORMRESIDUE(_Homography, _p0x, _p0y, _p0z,
						_p1x, _p1y, _p1z, _r0x, _r0y, _r0z);

				if (_errNorm) {
					*_errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
				}

				// compute the residue w.r.t the transformation with a delta
				// increment in each parameter
				for (int k=0; k<numParams; k++) {
					PERSTRANSFORMRESIDUE(mFwdTData[k], _p0x, _p0y, _p0z, _p1x, _p1y, _p1z,
							_r1[k*3], _r1[k*3+1], _r1[k*3+2]);
				}
				TIMERSTART(JtJJtErr);

				// compute the part of jacobian regarding this
				// point
				double *_r1_k = _r1;
				for (int k=0; k<numParams; k++){
					*_r1_k -= _r0x;
					*_r1_k *= scale;
					_r1_k++;
					*_r1_k -= _r0y;
					*_r1_k *= scale;
					_r1_k++;
					*_r1_k -= _r0z;
					*_r1_k *= scale;
					_r1_k++;

				}

				_r1_k = _r1;
				for (int k=0;
				k<numParams; k++) {
#if 1 // this branch is 1.5x to 2x faster than this branch below
		      // TODO: not quite sure about it. Would like to investigate more.
				  // @see LevMarqTransform::optimizeAlt
					double _r1x = *(_r1_k++);
					double _r1y = *(_r1_k++);
					double _r1z = *(_r1_k++);
					for (int l=k; l <numParams; l++) {
						// update the  JtJ entries
						JtJData[k*numParams + l] +=
							_r1x*_r1[l*3+0] + _r1y*_r1[l*3+1] + _r1z*_r1[l*3+2];
					}
					JtErrData[k] += _r1x*_r0x+_r1y*_r0y+_r1z*_r0z;
#else
					for (int c=0; c<3; c++) {
						double j = (_r1[k*3 + c]);
						JtErrData[k] += j*_r0[c];
						for (int l=k; l <numParams; l++) {
							// update the JtJ entries
							JtJData[k*numParams + l] += j*_r1[l*3 + c];
						}
					}
#endif
				}
				TIMEREND(JtJJtErr);
			}
			//	    	int64 tJtJJtErr = cvGetTickCount();
			for (int k=0; k<numParams; k++) {
				for (int l=k; l <numParams; l++) {
					// fill out the lower triangle just in case
					JtJData[l*numParams + k] = JtJData[k*numParams + l];
				}
			}
		}

		if (_errNorm) {
			if (_JtJ==NULL && _JtErr==NULL) {
    			double *p0 = xyzs0->data.db;
    			double *p1 = xyzs1->data.db;
				TIMERSTART(ErrNorm);
				// construct the transformation matrix
				constructTransformationMatrix(param0);
				for (int j=0; j<numPoints; j++) {
					// compute current error = xyzs1^T  - Transformation * xyzs0^T
    				double _r0x, _r0y, _r0z;
    				TIMERSTART2(Residue);
    				//	        	xyzs0 and xyzs1's are inliers we copy. so we know
    				//	        	how their data are organized
    				double _p0x = *p0++;
    				double _p0y = *p0++;
    				double _p0z = *p0++;

    				double _p1x = *p1++;
    				double _p1y = *p1++;
    				double _p1z = *p1++;
    				PERSTRANSFORMRESIDUE(_Homography, _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
    				TIMEREND2(Residue);

    				TIMERSTART(ErrNorm);
    				*_errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
    				TIMEREND(ErrNorm);
				}
				TIMEREND(ErrNorm);
			}
		}
	}
  // If we get out of the loop only because we reach the max times of updates
  // something is not quite right
  if (iterUpdates> defMaxTimesOfUpdates){
    cout << "Rearch the max number of iteration that jdc can tolerate"<<endl;
    double change = cvNorm(mLevMarq.param, mLevMarq.prevParam, CV_RELATIVE_L2);
    cout << "norm diff of param:" << change<<endl;
    cout << "error: "<< mLevMarq.errNorm<<","<<mLevMarq.prevErrNorm<<endl;
  }
	// now mLevMarq.params contains the solution.
	if (_param) {
		// copy the parameters out
		for (int i=0; i<numParams; i++) {
			_param[i] = cvmGet(mLevMarq.param, i, 0);
		}
	}

	return status;
}

