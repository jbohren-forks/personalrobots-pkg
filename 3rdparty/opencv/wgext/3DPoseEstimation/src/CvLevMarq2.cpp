#pragma warning( disable: 4996 )

#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>

#include "CvLevMarq2.h"

using namespace std;

#undef DEBUG

CvLevMarq2::CvLevMarq2()
{
   prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = 0;
   lambdaLg10 = 0; state = DONE;
   criteria = cvTermCriteria(0,0,0);
   iters = 0;
}

CvLevMarq2::CvLevMarq2( int nparams, int nerrs, CvTermCriteria criteria0 )
{
   prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = 0;
   init(nparams, nerrs, criteria0);
}

void CvLevMarq2::clear()
{
   cvReleaseMat(&prevParam);
   cvReleaseMat(&param);
   cvReleaseMat(&J);
   cvReleaseMat(&err);
   cvReleaseMat(&JtJ);
   cvReleaseMat(&JtJN);
   cvReleaseMat(&JtErr);
   cvReleaseMat(&JtJV);
   cvReleaseMat(&JtJW);
}

CvLevMarq2::~CvLevMarq2()
{
   clear();
}

void CvLevMarq2::init( int nparams, int nerrs, CvTermCriteria criteria0 )
{
   if( !param || param->rows != nparams || nerrs != (err ? err->rows : 0) )
       clear();
   prevParam = cvCreateMat( nparams, 1, CV_64F );
   param = cvCreateMat( nparams, 1, CV_64F );
   JtJ = cvCreateMat( nparams, nparams, CV_64F );
   JtJN = cvCreateMat( nparams, nparams, CV_64F );
   JtJV = cvCreateMat( nparams, nparams, CV_64F );
   JtJW = cvCreateMat( nparams, 1, CV_64F );
   JtErr = cvCreateMat( nparams, 1, CV_64F );
   if( nerrs > 0 )
   {
       J = cvCreateMat( nerrs, nparams, CV_64F );
       err = cvCreateMat( nerrs, 1, CV_64F );
   }
   prevErrNorm = DBL_MAX;
   lambdaLg10 = -3;
   criteria = criteria0;
   if( criteria.type & CV_TERMCRIT_ITER )
       criteria.max_iter = MIN(MAX(criteria.max_iter,1),1000);
   else
       criteria.max_iter = 30;
   if( criteria.type & CV_TERMCRIT_EPS )
       criteria.epsilon = MAX(criteria.epsilon, 0);
   else
       criteria.epsilon = DBL_EPSILON;
   state = STARTED;
   iters = 0;
}

bool CvLevMarq2::update( const CvMat*& _param, CvMat*& _J, CvMat*& _err )
{
   double change;

   assert( err != 0 );
   if( state == DONE )
   {
       _param = param;
       return false;
   }

   if( state == STARTED )
   {
       _param = param;
       _J = J;
       _err = err;
       state = CALC_J;
       return true;
   }

   if( state == CALC_J )
   {
       cvMulTransposed( J, JtJ, 1 );
       cvGEMM( J, err, 1, 0, 0, JtErr, CV_GEMM_A_T );
       cvCopy( param, prevParam );
       solve();
       cvSub( prevParam, param, param );
       if( iters == 0 )
           prevErrNorm = cvNorm(err, 0, CV_L2);
       _param = param;
       _err = err;
       state = CHECK_ERR;
       return true;
   }

   assert( state == CHECK_ERR );
   errNorm = cvNorm( err, 0, CV_L2 );
   // jdc debugging
//   if( errNorm >= prevErrNorm )
   if( errNorm > prevErrNorm )
   {
       lambdaLg10++;
       solve();
       cvSub( prevParam, param, param );
       _param = param;
       _err = err;
       state = CHECK_ERR;
       return true;
   }

   lambdaLg10 = MAX(lambdaLg10-1, -16);
   if( ++iters >= criteria.max_iter ||
       (change = cvNorm(param, prevParam, CV_RELATIVE_L2)) < criteria.epsilon )
   {
       _param = param;
       state = DONE;
       // jdc debugging
//       printf("cvLevMarq done: iter num: %d,  change: %f\n", iters, change);
       return false;
   }

   prevErrNorm = errNorm;
   _param = param;
   _J = J;
   state = CALC_J;
   return true;
}


bool CvLevMarq2::updateAlt( const CvMat*& _param, CvMat*& _JtJ, CvMat*& _JtErr, double*& _errNorm )
{
   double change;

   assert( err == 0 );
   if( state == DONE )
   {
       _param = param;
       return false;
   }

   if( state == STARTED )
   {
       _param = param;
       _JtJ = JtJ;
       _JtErr = JtErr;
       _errNorm = &errNorm;
       state = CALC_J;
       return true;
   }

   if( state == CALC_J )
   {
       cvCopy( param, prevParam );
       solve();
       cvSub( prevParam, param, param );
       _param = param;
       prevErrNorm = errNorm;
       _errNorm = &errNorm;
       state = CHECK_ERR;
       return true;
   }

   assert( state == CHECK_ERR );
   // jdc debugging
   //   if (errNorm - prevErrNorm > -0.1e-10)
   //   if( errNorm >= prevErrNorm )
   if( errNorm > prevErrNorm)
   {
       lambdaLg10++;
       solve();
       cvSub( prevParam, param, param );
       _param = param;
       _errNorm = &errNorm;
       state = CHECK_ERR;
       return true;
   }

   lambdaLg10 = MAX(lambdaLg10-1, -16);
   if( ++iters >= criteria.max_iter ||
       (change = cvNorm(param, prevParam, CV_RELATIVE_L2)) < criteria.epsilon )
   {
       _param = param;
       state = DONE;
#ifdef DEBUG
       // jdc debugging
       printf("cvLevMarq done: iter num: %d,  change: %f, errNorm: %f\n", iters, change, errNorm);
#endif
       return false;
   }
#ifdef DEBUG
   // jdc debugging
   printf("cvLevMarq continue: iter num: %d,  change: %le, %le, errNorm: %f\n",
		   iters, change, criteria.epsilon, errNorm);
#endif

   prevErrNorm = errNorm;
   _param = param;
   _JtJ = JtJ;
   _JtErr = JtErr;
   state = CALC_J;
   return true;
}

void CvLevMarq2::solve()
{
   const double LOG10 = log(10.);
   double lambda = exp(lambdaLg10*LOG10);

   cvSetIdentity( JtJN, cvRealScalar(lambda) );
   cvAdd( JtJ, JtJN, JtJN );
   cvSVD( JtJN, JtJW, 0, JtJV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
   cvSVBkSb( JtJW, JtJV, JtJV, JtErr, param, CV_SVD_U_T + CV_SVD_V_T );
#ifdef DEBUG
   // jdc debugging
   printf("param changed\n");
#endif
}

