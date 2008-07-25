#include <iostream>
using namespace std;

#include <cxcore.h>

#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvLevMarq3D.h"

#include "CvTestTimer.h"
#include <cv.h>

//#define DEBUG 1
#define USE_UPDATEALT
#define LAST3ISLIN

#if 0
#define TIMERSTART(x) 
#define TIMEREND(x)
#else
#define TIMERSTART(x) CvTestTimerStart(x)
#define TIMEREND(x) CvTestTimerEnd(x)
#endif


CvLevMarq3D::CvLevMarq3D(int numErrors, int numMaxIter)
{
	this->mAngleType = Euler;
	cvInitMatHeader(&mRT,         4, 4, CV_XF, mRTData);
	cvSetIdentity(&mRT);
	// get a view of the 3x4 transformation matrix that combines rot matrix and shift (translation) vector
	cvGetSubRect(&mRT, &mRT3x4, cvRect(0, 0, 4, 3));
	
	for (int i=0; i<numParams; i++){
		cvInitMatHeader(&(mFwdT[i]), 4, 4, CV_XF, mFwdTData[i]);
		cvSetIdentity(&(mFwdT[i]));
		// get a view of the 3x4 transformation matrix that combines rot matrix and shift (translation) vector
		cvGetSubRect(&mFwdT[i], &mFwdT3x4[i], cvRect(0, 0, 4, 3));
		
	}
	
#ifdef USE_UPDATEALT
	mLevMarq.init( numParams, 0, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,numMaxIter, DBL_EPSILON) );
	//numParams /* the number of parameters to optimize */, 
	//numErrors*3 /* dimensionality of the error vector */,
	//cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,numMaxIter,DBL_EPSILON)
	/* optional termination criteria (i.e. it stops when the number of iterations exceeds the specified limit or
	   when the change in the vector of parameters gets small enough */ 
#else
	cout << "numErrors in CvLevMarq3D:" << numErrors<<endl;
	mLevMarq.init(numParams, numErrors*3, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,numMaxIter, DBL_EPSILON) );
#endif
}

CvLevMarq3D::~CvLevMarq3D()
{
}

bool CvLevMarq3D::constructRTMatrices(const CvMat *param, CvMyReal delta) {
	CvMyReal x  = cvmGet(param, 0, 0);
	CvMyReal y  = cvmGet(param, 1, 0);
	CvMyReal z  = cvmGet(param, 2, 0);
	CvMyReal tx = cvmGet(param, 3, 0);
	CvMyReal ty = cvmGet(param, 4, 0);
	CvMyReal tz = cvmGet(param, 5, 0);
	
	CvMat3X3<CvMyReal>::transformMatrix(x, y, z, tx, ty, tz, mRTData, 4, CvMat3X3<CvMyReal>::XYZ);
	
	CvMyReal _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_XF, _param1);
	// transformation matrices for each parameter
	for (int k=0; k<numParams; k++) {
		cvCopy(param, &param1);
		_param1[k] += delta;
		constructRTMatrix(&param1, mFwdTData[k]);
	}
	return true;
}

bool CvLevMarq3D::constructRTMatrix(const CvMat * param, CvMyReal _RT[]){
	bool status = true;
	
	if (this->mAngleType == Euler) {
		
		CvMyReal x  = cvmGet(param, 0, 0);
		CvMyReal y  = cvmGet(param, 1, 0);
		CvMyReal z  = cvmGet(param, 2, 0);
		CvMyReal tx = cvmGet(param, 3, 0);
		CvMyReal ty = cvmGet(param, 4, 0);
		CvMyReal tz = cvmGet(param, 5, 0);

		CvMat3X3<CvMyReal>::transformMatrix(x, y, z, tx, ty, tz, _RT, 4, CvMat3X3<CvMyReal>::XYZ);
	} else {
	cout << "constructRTMatrix() Not Implemented Yet"<<endl;
	exit(0);
#if 0
		// Rodrigues
		CvMat rod;
		if (param->rows==1) {
			cvGetCols(param, &rod, 0, 3);
		} else {
			cvGetRows(param, &rod, 0, 3);
		}
		cvRodgrigues2(&rod, rot);
#endif
	}
	return status;
}

bool CvLevMarq3D::constructRTMatrix(const CvMat* param){
	bool status = true;
	
	double x = cvmGet(param, 0, 0);
	double y = cvmGet(param, 1, 0);
	double z = cvmGet(param, 2, 0);
	double _R[9];
	CvMat3X3<double>::rotMatrix(x, y, z, _R, CvMat3X3<double>::XYZ);
	
	cvSetReal2D(&mRT, 0, 0,  _R[0]);
	cvSetReal2D(&mRT, 0, 1,  _R[1]);
	cvSetReal2D(&mRT, 0, 2,  _R[2]);

	cvSetReal2D(&mRT, 1, 0,  _R[3]);
	cvSetReal2D(&mRT, 1, 1,  _R[4]);
	cvSetReal2D(&mRT, 1, 2,  _R[5]);

	cvSetReal2D(&mRT, 2, 0,  _R[6]);
	cvSetReal2D(&mRT, 2, 1,  _R[7]);
	cvSetReal2D(&mRT, 2, 2,  _R[8]);
	
	// translation vector
	cvSetReal2D(&mRT, 0, 3, cvmGet(param, 3, 0));
	cvSetReal2D(&mRT, 1, 3, cvmGet(param, 4, 0));
	cvSetReal2D(&mRT, 2, 3, cvmGet(param, 5, 0));
	
	// last row
	cvSetReal2D(&mRT, 3, 0, 0.);
	cvSetReal2D(&mRT, 3, 1, 0.);
	cvSetReal2D(&mRT, 3, 2, 0.);
	cvSetReal2D(&mRT, 3, 3, 1.);
	
#ifdef DEBUG
	cout << "CvLevMarq3D:: constructRTMatrix"<< endl;
	CvMatUtils::printMat(&mRT);
#endif
	
	return status;
}

bool CvLevMarq3D::computeResidue(CvMat* xyzs0, CvMat *xyzs1, CvMat* res){
	return computeResidue(xyzs0, xyzs1, &mRT3x4, res);
}
bool CvLevMarq3D::computeResidue(CvMat* xyzs0, CvMat *xyzs1, CvMat *T, CvMat* res){
	TIMERSTART(Residue);
	
	CvMat _xyzs0;
	CvMat _res;
	cvReshape(xyzs0, &_xyzs0, 3, 0);
	cvReshape(res, &_res, 3, 0);
	cvTransform(&_xyzs0, &_res, T);
	cvSub(res, xyzs1, res);
	
	TIMEREND(Residue);
	return true;
}

bool CvLevMarq3D::computeForwardResidues(CvMat *xyzs0, CvMat *xyzs1, CvMat *res){
	bool status = true;
	for (int k=0; k<numParams; k++) {
		CvMat r1_k;
		cvGetRow(res, &r1_k, k);
		computeResidue(xyzs0, xyzs1, &(mFwdT3x4[k]), &r1_k);
	}
	return status;
}

bool CvLevMarq3D::constructTransformationMatrix(const CvMat *param){
	return constructRTMatrix(param);
}

bool CvLevMarq3D::constructTransformationMatrix(const CvMat *param, CvMyReal T[]){
	return constructRTMatrix(param, T);
}

#if 1
bool CvLevMarq3D::constructTransformationMatrices(const CvMat *param, CvMyReal delta){
	return constructRTMatrices(param, delta);
}
#else

bool CvLevMarq3D::constructTransformationMatrices(const CvMat *param, CvMyReal delta) {
	CvMyReal x  = cvmGet(param, 0, 0);
	CvMyReal y  = cvmGet(param, 1, 0);
	CvMyReal z  = cvmGet(param, 2, 0);
	CvMyReal tx = cvmGet(param, 3, 0);
	CvMyReal ty = cvmGet(param, 4, 0);
	CvMyReal tz = cvmGet(param, 5, 0);
	
	CvMat3X3<CvMyReal>::transformMatrix(x, y, z, tx, ty, tz, mRTData, 4, CvMat3X3<CvMyReal>::XYZ);
	
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
#endif

bool CvLevMarq3D::doit(CvMat *xyzs0, CvMat *xyzs1, CvMat *rot, CvMat* trans) {
	bool status = true;
	double _param[6];
	CvMat rod;
	cvInitMatHeader(&rod, 1, 3, CV_64F, _param);
	// compute rodrigues
	cvRodrigues2(rot, &rod);
	this->mAngleType = Rodrigues;
	status = doit(xyzs0, xyzs1, _param);
	return status;
}

bool CvLevMarq3D::doit(CvMat *xyzs0, CvMat *xyzs1, double _param[]){
#ifdef USE_UPDATEALT
	return doit1(xyzs0, xyzs1, _param);
#else
	return doit2(xyzs0, xyzs1, _param);
#endif
}

bool CvLevMarq3D::doit1(CvMat *xyzs0, CvMat *xyzs1, double _param[]){
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
	
	double delta = CV_PI/(180.*10000.);
    
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
    		
    		TIMERSTART(ConstructMatrices);
    		// construct all the matirces need for JtJData JtErrData
	    	constructTransformationMatrices(param0, delta);
    		TIMEREND(ConstructMatrices);
	    	
#if 0
	    	CvMatUtils::printMat(&this->mRT);
	    	CvMatUtils::printMat(&this->mFwdT[0]);
	    	CvMatUtils::printMat(&this->mFwdT[1]);
	    	CvMatUtils::printMat(&this->mFwdT[2]);
	    	CvMatUtils::printMat(&this->mFwdT[3]);
	    	CvMatUtils::printMat(&this->mFwdT[4]);
	    	CvMatUtils::printMat(&this->mFwdT[5]);
#endif
	    	double *p0 = xyzs0->data.db;
	    	double *p1 = xyzs1->data.db;
	    	double errNorm = 0.0;
	    	for (int j=0; j<numPoints; j++) {
	        	// compute current error = xyzs1^T  - Transformation * xyzs0^T
	        	CvMat point0, point1;
	        	cvGetRow(xyzs0, &point0, j);
	        	cvGetRow(xyzs1, &point1, j);
	        	CvMyReal _r0x;
	        	CvMyReal _r0y;
	        	CvMyReal _r0z;
#if 0	        	
	        	computeResidue(&point0, &point1, &r0);
	        	_r0x = _r0[0];
	        	_r0y = _r0[1];
	        	_r0z = _r0[2];
#else
	        	CvMyReal _p0x, _p0y, _p0z,  _p1x, _p1y, _p1z;
	        	TIMERSTART(Residue);
//	        	xyzs0 and xyzs1's are inliers we copy. so we know
//	        	how their data are orgainized
#if 0
	        	_p0x = cvmGet(xyzs0, j, 0);
	        	_p0y = cvmGet(xyzs0, j, 1);
	        	_p0z = cvmGet(xyzs0, j, 2);
	        	
	        	_p1x = cvmGet(xyzs1, j, 0);
	        	_p1y = cvmGet(xyzs1, j, 1);
	        	_p1z = cvmGet(xyzs1, j, 2);	    
#else
	        	_p0x = *p0++;
	        	_p0y = *p0++;
	        	_p0z = *p0++;
	        	
	        	_p1x = *p1++;
	        	_p1y = *p1++;
	        	_p1z = *p1++;
#endif
	        	TRANSFORMRESIDUE(mRTData, _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
	        	TIMEREND(Residue);
#endif
	        	
	        	if (_errNorm) {
	        		errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
	        	}

	        	// TODO: skip the last 3 params
#ifdef LAST3ISLIN
	        	for (int k=0; k<3; k++) {
#else
	    		for (int k=0; k<numParams; k++) {
#endif
	    			CvMat r1_k;
	    			cvGetRow(&r1, &r1_k, k);
	    			
#if 0
	    			computeResidue(&point0, &point1, &mFwdT3x4[k], &r1_k);
#else
	    			TIMERSTART(Residue);
		        	TRANSFORMRESIDUE(mFwdTData[k], _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, 
		        			_r1[k*3], _r1[k*3+1], _r1[k*3+2]);
		        	TIMEREND(Residue);
#endif
	    		}
	    		
#ifdef DEBUG
	    		cout << "residues:"<<endl;
	    		cvMatUtils::printMat(&r0);
	    		cvMatUtils::printMat(&r1);
#endif
	    		

	    		TIMERSTART(JtJJtErr);
	    		
	    		// compute the part of jacobian regarding this
	    		// point
	    		CvMyReal *_r1_k = _r1;
#ifdef LAST3ISLIN
	    		for (int k=0; k<3; k++){
#else
	    		for (int k=0; k<numParams; k++){
#endif
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
	    		
#if 1 // this branch is 1.5x to 2x faster than this branch below
	    		_r1_k = _r1;
#ifdef LAST3ISLIN
	    		for (int k=0;	k<3; k++) {
	    			CvMyReal _r1x = *(_r1_k++);
	    			CvMyReal _r1y = *(_r1_k++);
	    			CvMyReal _r1z = *(_r1_k++);
	    			for (int l=k; l <3; l++) {
	    				// update the  JtJ entries
	    				JtJData[k*numParams + l] +=
	    					_r1x*_r1[l*3+0] + _r1y*_r1[l*3+1] + _r1z*_r1[l*3+2];
	    			}
	    			// TODO: not sure if the following is correct
	    			JtJData[k*numParams + 3] += _r1x;
	    			JtJData[k*numParams + 4] += _r1y;
	    			JtJData[k*numParams + 5] += _r1z;
	    			// update the JtErr entries
	    			JtErrData[k] += _r1x*_r0x+_r1y*_r0y+_r1z*_r0z;

	    		}
	    		// the last 3 entries
	    		JtErrData[3] += _r0x;
	    		JtErrData[4] += _r0y;
	    		JtErrData[5] += _r0z;
#else	    		
	    		for (int k=0;	k<numParams; k++) {
	    			CvMyReal _r1x = *(_r1_k++);
	    			CvMyReal _r1y = *(_r1_k++);
	    			CvMyReal _r1z = *(_r1_k++);
	    			for (int l=k; l <numParams; l++) {
	    				// update the  JtJ entries
	    				JtJData[k*numParams + l] +=
	    					_r1x*_r1[l*3+0] + _r1y*_r1[l*3+1] + _r1z*_r1[l*3+2];
	    			}
	    			// update the JtErr entries
	    			JtErrData[k] += _r1x*_r0x+_r1y*_r0y+_r1z*_r0z;
	    		}
#endif

#else 
	    		for (int k=0;	k<numParams; k++) {
	    			for (int c=0; c<3; c++) {
	    				CvMyReal j = (_r1[k*3 + c]);
	    				JtErrData[k] += j*_r0[c];
	    				for (int l=k; l <numParams; l++) {
	    					// update the JtJ entries
	    					JtJData[k*numParams + l] += j*_r1[l*3 + c];
	    				}
	    			}
	    		}
#endif
	    		
	    		TIMEREND(JtJJtErr);
	    	}

	    	if (_errNorm){
	    		*_errNorm = errNorm;
	    	}

#ifdef LAST3ISLIN
	    	for (int k=0; k<3; k++) {
	    		for (int l=k+1; l <numParams; l++) {
	    			// fill out the lower triangle just in case
	    			JtJData[l*numParams + k] = JtJData[k*numParams + l];
	    		}
	    	}
	    	JtJData[3*numParams + 3] = JtJData[4*numParams+4] = JtJData[5*numParams+5] = numPoints;
#else
	    	for (int k=0; k<numParams; k++) {
	    		for (int l=k+1; l <numParams; l++) {
	    			// fill out the lower triangle just in case
	    			JtJData[l*numParams + k] = JtJData[k*numParams + l];
	    		}
	    	}
#endif
	    	
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

	    		TIMERSTART(ConstructMatrices);
    			constructTransformationMatrix(param0);
        		TIMEREND(ConstructMatrices);
    			double *p0 = xyzs0->data.db;
    			double *p1 = xyzs1->data.db;
    			double errNorm=0.0;
    			for (int j=0; j<numPoints; j++) {
    				// compute current error = xyzs1^T  - Transformation * xyzs0^T
#if 0
    				CvMat point0, point1;
    				cvGetRow(xyzs0, &point0, j);
    				cvGetRow(xyzs1, &point1, j);
    				computeResidue(&point0, &point1, &r0);
		        	
    				*_errNorm += _r0[0]*_r0[0] + _r0[1]*_r0[1] + _r0[2]*_r0[2];
#else
    				double _r0x, _r0y, _r0z;
    				TIMERSTART(Residue);
    				//	        	xyzs0 and xyzs1's are inliers we copy. so we know
    				//	        	how their data are orgainized
#if 0
    				CvMyReal _p0x = cvmGet(xyzs0, j, 0);
    				CvMyReal _p0y = cvmGet(xyzs0, j, 1);
    				CvMyReal _p0z = cvmGet(xyzs0, j, 2);

    				CvMyReal _p1x = cvmGet(xyzs1, j, 0);
    				CvMyReal _p1y = cvmGet(xyzs1, j, 1);
    				CvMyReal _p1z = cvmGet(xyzs1, j, 2);
#else
    				CvMyReal _p0x = *p0++;
    				CvMyReal _p0y = *p0++;
    				CvMyReal _p0z = *p0++;

    				CvMyReal _p1x = *p1++;
    				CvMyReal _p1y = *p1++;
    				CvMyReal _p1z = *p1++;
#endif
    				TRANSFORMRESIDUE(mRTData, _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
    				TIMEREND(Residue);
		        	
    				errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
#endif
    			}
    			*_errNorm = errNorm;
	    		TIMEREND(ErrNorm);
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
	
bool CvLevMarq3D::doit2(CvMat *xyzs0, CvMat *xyzs1, double _param[]){
	cout << "CvLevMarq3D::doit2 --- Not Fixed Yet"<<endl;
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
	// two buffers to hold errors (residues)
	CvMat* errBuf0 = cvCreateMat(numPoints*3, 1, CV_64F);
	CvMat* errBuf1 = cvCreateMat(numPoints*3, 1, CV_64F);
//    CvMat* resVector  = cvCreateMat(4, numPoints, CV_64F);
//    CvMat* resVector1 = cvCreateMat(4, numPoints, CV_64F);
	
	CvMat* currErr = errBuf0;
	CvMat* currErr1 = errBuf1;
	
	double delta = CV_PI/(180.*100.);
    
	CvMyReal _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64FC1, _param1);
    
	for(int i=0;
        ;
		i++
	) {
	    CvMat *_param0=NULL, *J=NULL, *err=NULL;
		bool moreUpdate = mLevMarq.update( (const CvMat*&)_param0, J, err );
		if (moreUpdate == false) {
			break;
		}
#ifdef DEBUG
		cout << "iteration: "<<i<<endl;
#endif
		if (i > defMaxTimesOfUpdates){
			cout << "Rearch the max number of iteration that jdc can tolerate"<<endl;
			double change = cvNorm(mLevMarq.param, mLevMarq.prevParam, CV_RELATIVE_L2);
			cout << "norm diff of param:" << change<<endl;
			cout << "num of iter in cvLevMarq:" << mLevMarq.iters<<endl;
			break;
		}
#ifdef DEBUG
		if (_param0) {
			cout << "param: "<< endl;
			CvMatUtils::printMat(_param0);
		}
#endif
    	// construct the transformation matrix
    	constructTransformationMatrix(_param0);
    	// compute current error = xyzs1^T  - Transformation * xyzs0^T
#ifdef DEBUG
    	cout << "compute residue on curr param:"<<endl;
#endif

    	if (err) {
    		currErr = err;
    	} else {
    		currErr = errBuf0;
    	}
    	TIMERSTART(ErrNorm);
    	CvMat currRes;
    	cvReshape(currErr, &currRes, 0, numPoints);
    	computeResidue(xyzs0, xyzs1, &currRes);
    	TIMEREND(ErrNorm);

	    if( J )
	    {
	    	TIMERSTART(JtJJtErr);
	    	for (int k=0; k<numParams; k++){
	    		// compute the col of partial over param k
	    		cvCopy(_param0, &param1);
	    		cvmSet(&param1, k, 0, cvmGet(&param1, k, 0)+delta);
	    		constructTransformationMatrix(&param1);
	    		CvMat currRes1;
	    		cvReshape(currErr1, &currRes1, 0, numPoints);
	    		computeResidue(xyzs0, xyzs1, &currRes1);
	    		for (int j=0; j<numPoints; j++){
	    			for (int l=0; l<3; l++) {
//	    				cvmSet(currErr1, 3*j+l,   0, cvmGet(resVector1, l, j));
	    				double diff = cvmGet(currErr1, 3*j+l, 0) -
	    					cvmGet(currErr, 3*j+l, 0);
	    				cvmSet(J, 3*j+l, k, diff/delta);
	    			}
	    		}
	    	}
	    	TIMEREND(JtJJtErr);
#ifdef DEBUG
	    	cout << "Jacobian on iter: "<<i<<endl;
	    	CvMatUtils::printMat(J);
#endif
	    }
#ifdef DEBUG
	    printf("current parameters: %f, %f, %f, %f, %f, %f\n", 
			   cvmGet(mLevMarq.param, 0, 0)/CV_PI*180., 
			   cvmGet(mLevMarq.param, 1, 0)/CV_PI*180., 
			   cvmGet(mLevMarq.param, 2, 0)/CV_PI*180., 
			   cvmGet(mLevMarq.param, 3, 0), 
			   cvmGet(mLevMarq.param, 4, 0), 
			   cvmGet(mLevMarq.param, 5, 0));	  
#endif
	}
	// now solver.params contains the solution.	
	cvReleaseMat(&errBuf0);
	cvReleaseMat(&errBuf1);
    if (_param) {
        // copy the parameters out
        for (int i=0; i<numParams; i++) {
             _param[i] = cvmGet(mLevMarq.param, i, 0);
        }
    }   	
	return status;
}


