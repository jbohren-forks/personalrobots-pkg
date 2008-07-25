#include "CvLevMarqDispSpace.h"
#include <cxcore.h>
#include <iostream>
#include "CvMatUtils.h"
#include "CvMat3X3.h"
using namespace std;

#include "CvTestTimer.h"

//#define DEBUG 1

#if 0
#define TIMERSTART(x) 
#define TIMEREND(x)
#else
#define TIMERSTART(x) CvTestTimerStart(x)
#define TIMEREND(x) CvTestTimerEnd(x)
#endif

CvLevMarqDispSpace::CvLevMarqDispSpace(
            CvMat* disparityTo3D, CvMat *threeDToDisparity,
            int numErrors, int numMaxIter):
	CvLevMarq3D(
			numErrors /* dimensionality of the error vector */,
			numMaxIter
	), m3DToDisparity(threeDToDisparity), mDisparityTo3D(disparityTo3D)
{
	cvInitMatHeader(&mHomography, 4, 4, CV_64FC1, _Homography);
}

CvLevMarqDispSpace::~CvLevMarqDispSpace()
{
}


bool CvLevMarqDispSpace::constructHomographyMatrix(const CvMat* param){
	return constructHomographyMatrix(param, _Homography);
}

bool CvLevMarqDispSpace::constructHomographyMatrix(const CvMat* param, CvMyReal _H[]){
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
#ifdef DEBUG
	cout << "cvLevMarqDispSpace::constructHomographyMatrix"<<endl;
	CvMatUtils::printMat(&H);
	cout << "="<<endl;
	CvMatUtils::printMat(m3DToDisparity);
	CvMatUtils::printMat(&rt);
	CvMatUtils::printMat(mDisparityTo3D);
#endif
	return status;
}

bool CvLevMarqDispSpace::constructTransformationMatrix(const CvMat * param){
	return constructHomographyMatrix(param);
}

bool CvLevMarqDispSpace::constructTransformationMatrix(const CvMat * param, CvMyReal T[]) {
	return constructHomographyMatrix(param, T);
}

bool CvLevMarqDispSpace::constructTransformationMatrices(const CvMat *param, CvMyReal delta) {
//	CvMyReal x  = cvmGet(param, 0, 0);
//	CvMyReal y  = cvmGet(param, 1, 0);
//	CvMyReal z  = cvmGet(param, 2, 0);
//	CvMyReal tx = cvmGet(param, 3, 0);
//	CvMyReal ty = cvmGet(param, 4, 0);
//	CvMyReal tz = cvmGet(param, 5, 0);
	
	constructTransformationMatrix(param);
//	CvMat3X3<CvMyReal>::transformMatrix(x, y, z, tx, ty, tz, mRTData, 4, CvMat3X3<CvMyReal>::XYZ);
	
	CvMyReal _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_XF, _param1);
	// transformation matrices for each parameter
	for (int k=0; k<numParams; k++) {
		cvCopy(param, &param1);
		_param1[k] += delta;
		constructTransformationMatrix(&param1, mFwdTData[k]);
	}
	return true;
}


bool CvLevMarqDispSpace::computeResidueVector(const CvMat *uvdws0, const CvMat *uvdws1, 
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
#ifdef DEBUG
	cout << "residue" << endl;
	CvMatUtils::printMat(resVector);
#endif
	return true;
}

bool CvLevMarqDispSpace::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1, CvMat* res){
	return computeResidue(xyzs0, xyzs1, &mHomography, res);
}

bool CvLevMarqDispSpace::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1, 
		const CvMat *T, CvMat* res){
	TIMERSTART(Residue);
	CvMat _xyzs0;
	CvMat _res;
	cvReshape(xyzs0, &_xyzs0, 3, 0);
	cvReshape(res, &_res, 3, 0);
	cvPerspectiveTransform(&_xyzs0, &_res, T);
	cvSub(res, xyzs1, res);
#ifdef DEBUG
	cout << "residue" << endl;
	CvMatUtils::printMat(res);
#endif
	TIMEREND(Residue);
	return true;
}

// This function shall be the same as CvLevMarq3D::doit1 except for the 
// residue computation
bool CvLevMarqDispSpace::doit1(const CvMat *xyzs0, const CvMat *xyzs1, double _param[]){
	bool status=true;
	//initialize the initial vector of paramters
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

	CvMyReal _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64FC1, _param1);

	CvMyReal _r0[3];
	CvMat r0 = cvMat(1, 3, CV_64FC1, _r0);
	CvMyReal _r1[3*numParams];
	CvMat r1 = cvMat(numParams, 3, CV_64FC1, _r1);

#ifdef DEBUG
	//#if 1
	int numJtJComputed = 0;
#endif
	for(int i=0;
	;
	i++
	) {
		const CvMat *param0=NULL;
		CvMat *_JtJ=NULL;
		CvMat * _JtErr=NULL;
		double *_errNorm=NULL;
		bool moreUpdate;
		TIMERSTART(CvLevMarq)
		moreUpdate = mLevMarq.updateAlt(param0, 
				_JtJ, _JtErr, _errNorm );
		TIMEREND(CvLevMarq)
		if (moreUpdate == false) {
			break;
		}
#ifdef DEBUG
		cout << "iteration: "<<i<<endl;
#endif
		if (i> defMaxTimesOfUpdates){
			cout << "Rearch the max number of iteration that jdc can tolerate"<<endl;
			double change = cvNorm(mLevMarq.param, mLevMarq.prevParam, CV_RELATIVE_L2);
			cout << "norm diff of param:" << change<<endl;
			cout << "error: "<< mLevMarq.errNorm<<","<<mLevMarq.prevErrNorm<<endl;
			break;
		}
#ifdef DEBUG
		if (param0) {
			cout << "current param: "<< endl;
			CvMatUtils::printMat(param0);
		}
#endif

		if( _JtJ )
			cvZero( _JtJ );
		if( _JtErr )
			cvZero( _JtErr );
		if( _errNorm )
			*_errNorm = 0;

		if( _JtJ || _JtErr )
		{
#ifdef DEBUG
			//#if 1
			numJtJComputed++;
#endif
			CvMyReal scale = 1./delta;

			// Not sure if this is illegal;
			double* JtJData   = _JtJ->data.db;
			double* JtErrData = _JtErr->data.db;

			constructTransformationMatrices(param0, delta);

#ifdef DEBUG
			cout << "All the matrices Jacobian needs:"<<endl;
			CvMatUtils::printMat(&this->mHomography);
			CvMatUtils::printMat(&this->mFwdT[0]);
			CvMatUtils::printMat(&this->mFwdT[1]);
			CvMatUtils::printMat(&this->mFwdT[2]);
			CvMatUtils::printMat(&this->mFwdT[3]);
			CvMatUtils::printMat(&this->mFwdT[4]);
			CvMatUtils::printMat(&this->mFwdT[5]);
#endif

			for (int j=0; j<numPoints; j++) {
				// compute current error = xyzs1^T  - Transformation * xyzs0^T
				CvMat point0, point1;
				cvGetRow(xyzs0, &point0, j);
				cvGetRow(xyzs1, &point1, j);
				CvMyReal _r0x;
				CvMyReal _r0y;
				CvMyReal _r0z;
#if 1	        	
				computeResidue(&point0, &point1, &r0);
				_r0x = _r0[0];
				_r0y = _r0[1];
				_r0z = _r0[2];
#else
				//	        	xyzs0 and xyzs1's are inliers we copy. so we know
				//	        	how their data are orgainized
				CvMyReal _p0x = cvmGet(xyzs0, j, 0);
				CvMyReal _p0y = cvmGet(xyzs0, j, 1);
				CvMyReal _p0z = cvmGet(xyzs0, j, 2);

				CvMyReal _p1x = cvmGet(xyzs1, j, 0);
				CvMyReal _p1y = cvmGet(xyzs1, j, 1);
				CvMyReal _p1z = cvmGet(xyzs1, j, 2);	        	
				PERSTRANSFORMRESIDUE(_Homography, _p0x, _p0y, _p0z, 
						_p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
#endif

				if (_errNorm) {
					*_errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
				}

				// TODO: compute transformation matrix of all diff directions
				for (int k=0; k<numParams; k++) {
					CvMat r1_k;
					cvGetRow(&r1, &r1_k, k);

#if 1
					computeResidue(&point0, &point1, &mFwdT[k], &r1_k);
#else
					PERSTRANSFORMRESIDUE(mFwdTData[k], _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, 
							_r1[k*3], _r1[k*3+1], _r1[k*3+2]);
#endif
				}
				TIMERSTART(JtJJtErr);
				//	    		int64 tJtJJtErr = cvGetTickCount();

				// compute the part of jacobian regarding this
				// point
				CvMyReal *_r1_k = _r1;
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

#if 0
					for (int c=0; c<3; c++){
						_r1[k*3+c] -= _r0[c];
						_r1[k*3+c] *= scale;
					}
#endif
				}

				_r1_k = _r1;
				for (int k=0;
				k<numParams; k++) {
#if 1 // this branch is 1.5x to 2x faster than this branch below
					CvMyReal _r1x = *(_r1_k++);
					CvMyReal _r1y = *(_r1_k++);
					CvMyReal _r1z = *(_r1_k++);
					for (int l=k; l <numParams; l++) {
						// update the  JtJ entries
						JtJData[k*numParams + l] +=
							_r1x*_r1[l*3+0] + _r1y*_r1[l*3+1] + _r1z*_r1[l*3+2];
					}
					JtErrData[k] += _r1x*_r0x+_r1y*_r0y+_r1z*_r0z;
#else 
					for (int c=0; c<3; c++) {
						CvMyReal j = (_r1[k*3 + c]);
						JtErrData[k] += j*_r0[c];
						for (int l=k; l <numParams; l++) {
							// update the JtJ entries
							JtJData[k*numParams + l] += j*_r1[l*3 + c];
						}
					}
#endif
				}
				CvTestTimerEnd(JtJJtErr);
				//	    		CvTestTimer::getTimer().mJtJJtErr += cvGetTickCount() - tJtJJtErr;
			}
			//	    	int64 tJtJJtErr = cvGetTickCount();
			for (int k=0; k<numParams; k++) {
				for (int l=k; l <numParams; l++) {
					// fill out the lower triangle just in case
					JtJData[l*numParams + k] = JtJData[k*numParams + l];
				}
			}
			//	    	CvTestTimer::getTimer().mJtJJtErr += cvGetTickCount() - tJtJJtErr;
#ifdef DEBUG
			cout << "JtJ on iter: "<<i<<endl;
			CvMatUtils::printMat(_JtJ);
			cout << "JtErr on iter: "<<i<<endl;
			CvMatUtils::printMat(_JtErr);
#endif
		}

		if (_errNorm) {
			if (_JtJ==NULL && _JtErr==NULL) {
				TIMERSTART(ErrNorm);
				//    			int64 tErrNorm = cvGetTickCount();
				// not computed yet
				// construct the transformation matrix
				constructTransformationMatrix(param0);
				for (int j=0; j<numPoints; j++) {
					// compute current error = xyzs1^T  - Transformation * xyzs0^T
					CvMat point0, point1;
					cvGetRow(xyzs0, &point0, j);
					cvGetRow(xyzs1, &point1, j);
					computeResidue(&point0, &point1, &r0);

					*_errNorm += _r0[0]*_r0[0] + _r0[1]*_r0[1] + _r0[2]*_r0[2];
				}
				TIMEREND(ErrNorm);
				//    			CvTestTimer::getTimer().mErrNorm += cvGetTickCount() - tErrNorm;
			}
		}
#ifdef DEBUG
		printf("current parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n", 
				cvmGet(mLevMarq.param, 0, 0), cvmGet(mLevMarq.param, 0, 0)/CV_PI*180., 
				cvmGet(mLevMarq.param, 1, 0), cvmGet(mLevMarq.param, 1, 0)/CV_PI*180., 
				cvmGet(mLevMarq.param, 2, 0), cvmGet(mLevMarq.param, 2, 0)/CV_PI*180., 
				cvmGet(mLevMarq.param, 3, 0), 
				cvmGet(mLevMarq.param, 4, 0), 
				cvmGet(mLevMarq.param, 5, 0));
#endif
	}
#ifdef DEBUG
	fprintf(stdout, "Num of JtJ computed: %d\n", numJtJComputed);
	fprintf(stdout, "Num of iteration with LevMarq.update(): %d\n", mLevMarq.iters);
#endif
	// now mLevMarq.params contains the solution.	
	if (_param) {
		// copy the parameters out
		for (int i=0; i<numParams; i++) {
			_param[i] = cvmGet(mLevMarq.param, i, 0);
		}
	}

	return status;
}
