#include <iostream>
using namespace std;

#include <opencv/cxcore.h>
#include <opencv/cv.h>

#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "LevMarqTransform.h"

#include "CvTestTimer.h"

using namespace cv::willow;

#undef DEBUG

// define the following if the last three parameters is linear (e.g. shift parameters),
// so as to speed up by skipping over unnecessary computation

// Please note that because the timing code is executed is called lots of lots of times
// they themselves have taken substantial timing as well
#define CHECKTIMING 0

#if CHECKTIMING == 0
#define TIMERSTART(x)
#define TIMEREND(x)
#define TIMERSTART2(x)
#define TIMEREND2(x)
#else
#define TIMERSTART(x)  CvTestTimerStart(x)
#define TIMEREND(x)    CvTestTimerEnd(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x)   CvTestTimerEnd2(x)
#endif


LevMarqTransform::LevMarqTransform(int numMaxIter, AngleType angleType):
  mAngleType(angleType)
{
	cvInitMatHeader(&mRT,         4, 4, CV_64F, mRTData);
	cvSetIdentity(&mRT);
	// get a view of the 3x4 transformation matrix that combines rot matrix and shift (translation) vector
	cvGetSubRect(&mRT, &mRT3x4, cvRect(0, 0, 4, 3));

	for (int i=0; i<numParams; i++){
		cvInitMatHeader(&(mFwdT[i]), 4, 4, CV_64F, mFwdTData[i]);
		cvSetIdentity(&(mFwdT[i]));
		// get a view of the 3x4 transformation matrix that combines rot matrix and shift (translation) vector
		cvGetSubRect(&mFwdT[i], &mFwdT3x4[i], cvRect(0, 0, 4, 3));

	}

	mLevMarq.init( numParams /* the number of parameters to optimize */,
	    0  /* dimensionality of the error vector, not needed as updateAlt is used */,
	    cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,numMaxIter, DBL_EPSILON)
	    /* optional termination criteria (i.e. it stops when the number of iterations exceeds the specified limit or
				   when the change in the vector of parameters gets small enough */
	);
}

LevMarqTransform::~LevMarqTransform()
{
}

bool LevMarqTransform::constructRTMatrices(const CvMat *param, double delta) {
  constructRTMatrix(param, mRTData);

	double _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64F, _param1);
	// transformation matrices for each parameter
	for (int k=0; k<numParams; k++) {
		cvCopy(param, &param1);
		_param1[k] += delta;
		constructRTMatrix(&param1, mFwdTData[k]);
	}
	return true;
}

bool LevMarqTransform::constructRTMatrix(const CvMat * param, double _RT[]) const {
	bool status = true;

	switch(mAngleType) {
	case Euler:
	{
	  double x  = cvmGet(param, 0, 0);
		double y  = cvmGet(param, 1, 0);
		double z  = cvmGet(param, 2, 0);
		double tx = cvmGet(param, 3, 0);
		double ty = cvmGet(param, 4, 0);
		double tz = cvmGet(param, 5, 0);

		CvMat3X3<double>::transformMatrix(x, y, z, tx, ty, tz, _RT, 4, CvMat3X3<double>::EulerXYZ);
		break;
	}
	case Rodrigues:
	{
    CvMat rt;
    cvInitMatHeader(&rt, 4, 4, CV_64FC1, _RT);
		CvMatUtils::TransformationFromRodriguesAndShift(*param, rt);
		break;
	}
	default:
		cout << "constructRTMatrix() Not Implemented Yet"<<endl;
		exit(0);
	}

	return status;
}

bool LevMarqTransform::constructRTMatrix(const CvMat* param){
  return constructRTMatrix(param, mRTData);
}

bool LevMarqTransform::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1, CvMat* res){
	return computeResidue(xyzs0, xyzs1, &mRT3x4, res);
}
bool LevMarqTransform::computeResidue(const CvMat* xyzs0, const CvMat *xyzs1, const CvMat *T, CvMat* res){
	TIMERSTART2(Residue);

	CvMat _xyzs0;
	CvMat _res;
	cvReshape(xyzs0, &_xyzs0, 3, 0);
	cvReshape(res, &_res, 3, 0);
	cvTransform(&_xyzs0, &_res, T);
	cvSub(res, xyzs1, res);

	TIMEREND2(Residue);
	return true;
}

bool LevMarqTransform::computeForwardResidues(const CvMat *xyzs0, const CvMat *xyzs1, CvMat *res){
	bool status = true;
	for (int k=0; k<numParams; k++) {
		CvMat r1_k;
		cvGetRow(res, &r1_k, k);
		computeResidue(xyzs0, xyzs1, &(mFwdT3x4[k]), &r1_k);
	}
	return status;
}

bool LevMarqTransform::constructTransformationMatrix(const CvMat *param){
	return constructRTMatrix(param);
}

bool LevMarqTransform::constructTransformationMatrix(const CvMat *param, double T[]){
	return constructRTMatrix(param, T);
}

bool LevMarqTransform::constructTransformationMatrices(const CvMat *param, double delta){
	return constructRTMatrices(param, delta);
}

bool LevMarqTransform::optimize(const CvMat *xyzs0, const CvMat *xyzs1, CvMat *rot, CvMat* trans) {
	bool status = true;
	double _param[6];
	CvMat rod;
	// the first 3 entries in param are the rodrigues rotation params
	cvInitMatHeader(&rod, 1, 3, CV_64F, _param);
	// compute rodrigues
	cvRodrigues2(rot, &rod);
//	this->mAngleType = Rodrigues;
	status = optimize(xyzs0, xyzs1, _param);
	return status;
}

bool LevMarqTransform::optimize(const CvMat *xyzs0, const CvMat *xyzs1, double _param[]){
	// if xyzs0 or xyzs1 is not of a CvMat of CV_64FC1, with step = 64*3, we allocate two buffers and
	// convert data into such
	if (CV_IS_MAT_HDR(xyzs0)==false || CV_IS_MAT_HDR(xyzs1)==false ||
			CV_ARE_SIZES_EQ(xyzs0, xyzs1)==false ||
			CV_ARE_CNS_EQ(xyzs0, xyzs1) == false) {
		cerr << "input matrices are either not of the correct type or not matched"<<endl;
		return false;
	}
	bool status = true;
	CvMat* _xyzs0 = NULL;
	CvMat* _xyzs1 = NULL;

	if (xyzs0->step != 64*3 ||  CV_MAT_TYPE(xyzs0->type) != CV_64FC1) {
		// make a buffer and convert the input points into the right type
		_xyzs0 = cvCreateMat(xyzs0->rows, xyzs0->cols, CV_64FC1);
		cvConvert(xyzs0, _xyzs0);
	}
	if (xyzs1->step != 64*3 ||  CV_MAT_TYPE(xyzs1->type) != CV_64FC1) {
		// make a buffer and convert the input points into the right type
		_xyzs1 = cvCreateMat(xyzs1->rows, xyzs1->cols, CV_64FC1);
		cvConvert(xyzs1, _xyzs1);
	}
	status = optimizeAlt(_xyzs0, _xyzs1, _param);

	if (_xyzs0) {
		cvReleaseMat(&_xyzs0);
	}
	if (_xyzs1) {
		cvReleaseMat(&_xyzs1);
	}
	return status;
}

bool LevMarqTransform::optimizeAlt(const CvMat *xyzs0,
		const CvMat *xyzs1, double _param[]){
	bool status=true;
	TIMERSTART2(LevMarq2);
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

	double delta = CV_PI/(180.*10000.);

	double _param1[numParams];
	CvMat param1 = cvMat(numParams, 1, CV_64FC1, _param1);

	// buffer for forward differences of the current point in x,y,z w.r.t all parameters
	double _r1[3*numParams];
	CvMat r1 = cvMat(numParams, 3, CV_64FC1, _r1);

	int iterUpdates;
	for(iterUpdates=0;	iterUpdates <= defMaxTimesOfUpdates;	iterUpdates++
	) {
	    const CvMat *param0=NULL;
	    CvMat *_JtJ=NULL;
	    CvMat * _JtErr=NULL;
	    double *_errNorm=NULL;
	    bool moreUpdate;
	    TIMERSTART(LevMarq);
	    moreUpdate = mLevMarq.updateAlt(param0, _JtJ, _JtErr, _errNorm );
	    TIMEREND(LevMarq)

	    TIMERSTART2(LevMarq3);
	    if (moreUpdate == false) {
	      break;
	    }

	    if( _JtJ )
	      cvZero( _JtJ );
	    if( _JtErr )
	      cvZero( _JtErr );
	    if( _errNorm )
	      *_errNorm = 0;

	    TIMERSTART2(LevMarq4);
	    if( _JtJ || _JtErr )
	    {
	      double scale = 1./delta;

	      // Not sure if this is illegal;
	      double* JtJData   = _JtJ->data.db;
	      double* JtErrData = _JtErr->data.db;

    		TIMERSTART(ConstructMatrices);
    		// construct all the matrices need for JtJData JtErrData
	    	constructTransformationMatrices(param0, delta);
    		TIMEREND(ConstructMatrices);

        	//	xyzs0 and xyzs1's are data point we copy. so we know
        	//	how their data are organized
	    	double *p0 = xyzs0->data.db;
	    	double *p1 = xyzs1->data.db;
	    	double errNorm = 0.0;
	    	// go thru each points and compute current error as
        	//  error = xyzs1^T  - Transformation * xyzs0^T
	    	for (int j=0; j<numPoints; j++) {
	        	double _r0x;	// residue of the current point in x dim
	        	double _r0y;  // residue of the current point in y dim
	        	double _r0z;  // residue of the current point in z dim
	        	TIMERSTART2(Residue);
	        	// no sure about the order evaluation in the macro. We store
	        	// the value in a local variable.
	        	// TODO: use reference instead?
	        	double _p0x = *p0++;	// xyzs0[j, 0]
	        	double _p0y = *p0++;	// xyzs0[j, 1]
	        	double _p0z = *p0++;	// xyzs0[j, 2]

	        	double _p1x = *p1++;	// xyzs1[j, 0]
	        	double _p1y = *p1++;	// xyzs1[j, 1]
	        	double _p1z = *p1++;	// xyzs1[j, 2]
	        	TRANSFORMRESIDUE(mRTData, _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
	        	TIMEREND2(Residue);

	        	if (_errNorm) {
	        		errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
	        	}

	        	// compute the residues w.r.t. the forwarded parameters in
	        	// each of the 6 component
	        	// skip the last 3 params as they are linear, and so can be
	        	// computed without going thru the coords of the points.
	        	for (int k=0; k<numNonLinearParams; k++) {
	    			TIMERSTART2(FwdResidue);
		        	TRANSFORMRESIDUE(mFwdTData[k], _p0x, _p0y, _p0z, _p1x, _p1y, _p1z,
		        			_r1[k*3], _r1[k*3+1], _r1[k*3+2]);
		        	TIMEREND2(FwdResidue);
	    		}

	    		TIMERSTART(JtJJtErr);

	    		// Compute the part of jacobian regarding this point
	    		// Each entry JtJ[s,t] is the dot product of column s and column t of J, or
	    		// in other words, the sum of the product of errors of point i in a specific dimension
	    		// (e.g. x) w.r.t parameters s and t
	    		// or \sum_{i=0, d=\{x,y,z\}}^n (\partial E_{i,d} / \partial s) (\partial E_{i,d} / \partial t)
	    		// so in each iteration we update JtJ with the contribution of point i
	    		// into the sum

	    		// approximate the partial derivatives of the errors in x, y and z, w.r.t. the nonlinear
	    		// parameters with forward differences.
	    		double *_r1_k = _r1;
	    		for (int k=0; k<numNonLinearParams; k++){
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

#if 1 // this branch is 1.5x to 2x faster than this branch below. But I am not convinced why
	    		// update each JtJ entry with contribution from this point, a sum of
	    		// product of partial in x,y,z w.r.t. parameters
	    		// the entries involve linear parameters are filled out efficiently by exploiting
	    		// the sparsity of the matrix
	    		// Similar treatment is applied to JtJErr update.
	        // TODO: not quite sure about it. Would like to investigate more.
	    		_r1_k = _r1;
	    		assert(numNonLinearParams == 3);
	    		for (int k=0;	k<numNonLinearParams; k++) {
	    			double _r1x = *(_r1_k++);
	    			double _r1y = *(_r1_k++);
	    			double _r1z = *(_r1_k++);
	    			for (int l=k; l <numNonLinearParams; l++) {
	    				// update the  JtJ entries w.r.t nonlinear parameters
	    				JtJData[k*numParams + l] +=
	    					_r1x*_r1[l*3+0] + _r1y*_r1[l*3+1] + _r1z*_r1[l*3+2];
	    			}
	    			// we already know that the partial w.r.t. the linear parameters
	    			// are [1,0,0], [0,1,0], and [0,0,1] respectively.
	    			JtJData[k*numParams + 3] += _r1x;
	    			JtJData[k*numParams + 4] += _r1y;
	    			JtJData[k*numParams + 5] += _r1z;
	    			// update the JtErr entries
	    			JtErrData[k] += _r1x*_r0x+_r1y*_r0y+_r1z*_r0z;

	    		}
	    		// the last 3 entries, w.r.t the linear parameters
	    		JtErrData[3] += _r0x;
	    		JtErrData[4] += _r0y;
	    		JtErrData[5] += _r0z;

#else
	    		for (int k=0;	k<numParams; k++) {
	    			for (int c=0; c<3; c++) {
	    				double j = (_r1[k*3 + c]);
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

			// fill out the lower triangle just in case
	    	for (int k=0; k<numNonLinearParams; k++) {
	    		for (int l=k+1; l <numParams; l++) {
	    			JtJData[l*numParams + k] = JtJData[k*numParams + l];
	    		}
	    	}
	    	// fill out the diagonal elements of the lower right square w.r.t the linear parameters
	    	// As the Jacobian sub matrix is an identity, the JtJ entries are merely the sums of 1's
	    	// or the number of points.
	    	assert(numNonLinearParams == 3);
	    	JtJData[3*numParams + 3] = JtJData[4*numParams+4] = JtJData[5*numParams+5] = numPoints;
	    }
	    TIMEREND2(LevMarq4);
	    TIMERSTART2(LevMarq5);

    	if (_errNorm) {
    		if (_JtJ==NULL && _JtErr==NULL) {

	    		TIMERSTART(ConstructMatrices);
    			constructTransformationMatrix(param0);
    			TIMEREND(ConstructMatrices);
    			// xyzs0 and xyzs1's are inliers we copy. so we know
    			// how their data are organized
    			double *p0 = xyzs0->data.db;
    			double *p1 = xyzs1->data.db;
    			double errNorm=0.0;
    			for (int j=0; j<numPoints; j++) {
    				// compute current error = xyzs1^T  - Transformation * xyzs0^T
    				double _r0x, _r0y, _r0z;
    				TIMERSTART2(Residue);
    				// no sure about the order evaluation in the macro. We store
    				// the value in a local variable.
    				// TODO: use reference instead?
    				double _p0x = *p0++;  // cvmGet(xyzs0, j, 0);
    				double _p0y = *p0++;	// cvmGet(xyzs0, j, 1);
    				double _p0z = *p0++;	// cvmGet(xyzs0, j, 2);

    				double _p1x = *p1++;	// cvmGet(xyzs1, j, 0);
    				double _p1y = *p1++;	// cvmGet(xyzs1, j, 1);
    				double _p1z = *p1++;	// cvmGet(xyzs1, j, 2);

    				TRANSFORMRESIDUE(mRTData, _p0x, _p0y, _p0z, _p1x, _p1y, _p1z, _r0x, _r0y, _r0z);
    				TIMEREND2(Residue);

        			TIMERSTART(ErrNorm);
    				errNorm += _r0x*_r0x + _r0y*_r0y + _r0z*_r0z;
    	    		TIMEREND(ErrNorm);
    			}
    			*_errNorm = errNorm;
    		}
    	}
	    TIMEREND2(LevMarq5);
	    TIMEREND2(LevMarq3);
	}

	// If we get out of the loop only because we reach the max times of updates
	// something is not quite right
  if (iterUpdates> defMaxTimesOfUpdates){
    cout << "Reach the max number of iteration that jdc can tolerate"<<endl;
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
	TIMEREND2(LevMarq2);
	return status;
}

void LevMarqTransform::rotAndShiftMatsToParams(const CvMat& rot, const CvMat& trans,
    double param[6]) const {
  switch(mAngleType) {
  case LevMarqTransform::Euler: {
    // get the euler angle from rot
    CvPoint3D64f eulerAngles;
    {
      double _R[9], _Q[9];
      CvMat R, Q;
      CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
      cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
      cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

      cvRQDecomp3x3(&rot, &R, &Q, pQx, pQy, pQz, &eulerAngles);
    }
    param[0] = eulerAngles.x/180. * CV_PI;
    param[1] = eulerAngles.y/180. * CV_PI;
    param[2] = eulerAngles.z/180. * CV_PI;
    param[3] = cvmGet(&trans, 0, 0);
    param[4] = cvmGet(&trans, 1, 0);
    param[5] = cvmGet(&trans, 2, 0);
    break;
  }
  case LevMarqTransform::Rodrigues: {
    CvMat rod = cvMat(1, 3, CV_64FC1, param);
    cvRodrigues2(&rot, &rod);
    param[3] = cvmGet(&trans, 0, 0);
    param[4] = cvmGet(&trans, 1, 0);
    param[5] = cvmGet(&trans, 2, 0);
    break;
  }
  default:
    cerr << "unknow angle type"<<endl;
  }
}

void LevMarqTransform::paramsToRotAndShiftMats(const double param[6],
    CvMat& rot, CvMat& trans) const {
  switch(mAngleType) {
  case LevMarqTransform::Euler: {
    double _rot[9];
    CvMat rot0 = cvMat(3, 3, CV_64FC1, _rot);
    CvMat3X3<double>::rotMatrix(param[0], param[1], param[2], _rot,
        CvMat3X3<double>::EulerXYZ);
    cvCopy(&rot0, &rot);

    cvmSet(&trans, 0, 0, param[3]);
    cvmSet(&trans, 1, 0, param[4]);
    cvmSet(&trans, 2, 0, param[5]);

    break;
  }
  case LevMarqTransform::Rodrigues: {
    CvMat rod = cvMat(1, 3, CV_64FC1, (void *)param);
    cvRodrigues2(&rod, &rot);

    cvmSet(&trans, 0, 0, param[3]);
    cvmSet(&trans, 1, 0, param[4]);
    cvmSet(&trans, 2, 0, param[5]);
    break;
  }
  default:
    cerr << "unknow angle type"<<endl;
  }
}
