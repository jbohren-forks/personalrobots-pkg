#ifndef LEVMARQ_H_
#define LEVMARQ_H_

#include "cxtypes.h"

struct CvLevMarq
{
   CvLevMarq();
   CvLevMarq( int nparams, int nerrs, CvTermCriteria criteria=
       cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON) );
   ~CvLevMarq();
   void init( int nparams, int nerrs, CvTermCriteria criteria=
       cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON) );
   bool update( const CvMat*& param, CvMat*& J, CvMat*& err );
   bool updateAlt( const CvMat*& param, CvMat*& JtJ, CvMat*& JtErr, double*& errNorm );

   void clear();
   void solve();
   enum { DONE=0, STARTED=1, CALC_J=2, CHECK_ERR=3 };

   CvMat* prevParam;
   CvMat* param;
   CvMat* J;
   CvMat* err;
   CvMat* JtJ;
   CvMat* JtJN;
   CvMat* JtErr;
   CvMat* JtJV;
   CvMat* JtJW;
   double prevErrNorm, errNorm;
   int lambdaLg10;
   CvTermCriteria criteria;
   int state;
   int iters;
};

#endif /*LEVMARQ_H_*/
