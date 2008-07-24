#pragma warning( disable: 4996 )

#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>

#include "calib_stereo.h"

using namespace std;

// reimplementation of dAB.m
static void
cvCalcMatMulDeriv( const CvMat* A, const CvMat* B, CvMat* dABdA, CvMat* dABdB )
{
   CV_FUNCNAME( "cvCalcMatMulDeriv" );

   __BEGIN__;

   int i, j, M, N, L;
   int bstep;

   CV_ASSERT( CV_IS_MAT(A) && CV_IS_MAT(B) );
   CV_ASSERT( CV_ARE_TYPES_EQ(A, B) &&
       (CV_MAT_TYPE(A->type) == CV_32F || CV_MAT_TYPE(A->type) == CV_64F) );
   CV_ASSERT( A->cols == B->rows );

   M = A->rows;
   L = A->cols;
   N = B->cols;
   bstep = B->step/CV_ELEM_SIZE(B->type);

   if( dABdA )
   {
       CV_ASSERT( CV_ARE_TYPES_EQ(A, dABdA) &&
           dABdA->rows == A->rows*B->cols && dABdA->cols == A->rows*A->cols );
   }

   if( dABdB )
   {
       CV_ASSERT( CV_ARE_TYPES_EQ(A, dABdB) &&
           dABdB->rows == A->rows*B->cols && dABdB->cols == B->rows*B->cols );
   }

   if( CV_MAT_TYPE(A->type) == CV_32F )
   {
       for( i = 0; i < M*N; i++ )
       {
           int i1 = i / N,  i2 = i % N;

           if( dABdA )
           {
               float* dcda = (float*)(dABdA->data.ptr + dABdA->step*i);
               const float* b = (const float*)B->data.ptr + i2;

               for( j = 0; j < M*L; j++ )
                   dcda[j] = 0;
               for( j = 0; j < L; j++ )
                   dcda[i1*L + j] = b[j*bstep];
           }

           if( dABdB )
           {
               float* dcdb = (float*)(dABdB->data.ptr + dABdB->step*i);
               const float* a = (const float*)(A->data.ptr + A->step*i1);

               for( j = 0; j < L*N; j++ )
                   dcdb[j] = 0;
               for( j = 0; j < L; j++ )
                   dcdb[j*N + i2] = a[j];
           }
       }
   }
   else
   {
       for( i = 0; i < M*N; i++ )
       {
           int i1 = i / N,  i2 = i % N;

           if( dABdA )
           {
               double* dcda = (double*)(dABdA->data.ptr + dABdA->step*i);
               const double* b = (const double*)B->data.ptr + i2;

               for( j = 0; j < M*L; j++ )
                   dcda[j] = 0;
               for( j = 0; j < L; j++ )
                   dcda[i1*L + j] = b[j*bstep];
           }

           if( dABdB )
           {
               double* dcdb = (double*)(dABdB->data.ptr + dABdB->step*i);
               const double* a = (const double*)(A->data.ptr + A->step*i1);

               for( j = 0; j < L*N; j++ )
                   dcdb[j] = 0;
               for( j = 0; j < L; j++ )
                   dcdb[j*N + i2] = a[j];
           }
       }
   }

   __END__;
}

// reimplementation of compose_motion.m
static void
cvComposeRT( const CvMat* _rvec1, const CvMat* _tvec1,
            const CvMat* _rvec2, const CvMat* _tvec2,
            CvMat* _rvec3, CvMat* _tvec3,
            CvMat* dr3dr1=0, CvMat* dr3dt1=0,
            CvMat* dr3dr2=0, CvMat* dr3dt2=0,
            CvMat* dt3dr1=0, CvMat* dt3dt1=0,
            CvMat* dt3dr2=0, CvMat* dt3dt2=0 )
{
   CV_FUNCNAME( "cvComposeRT" );

   __BEGIN__;

   double _r1[3], _r2[3];
   double _R1[9], _d1[9*3], _R2[9], _d2[9*3];
   CvMat r1 = cvMat(3,1,CV_64F,_r1), r2 = cvMat(3,1,CV_64F,_r2);
   CvMat R1 = cvMat(3,3,CV_64F,_R1), R2 = cvMat(3,3,CV_64F,_R2);
   CvMat dR1dr1 = cvMat(9,3,CV_64F,_d1), dR2dr2 = cvMat(9,3,CV_64F,_d2);

   CV_ASSERT( CV_IS_MAT(_rvec1) && CV_IS_MAT(_rvec2) );

   CV_ASSERT( CV_MAT_TYPE(_rvec1->type) == CV_32F ||
              CV_MAT_TYPE(_rvec1->type) == CV_64F );

   CV_ASSERT( _rvec1->rows == 3 && _rvec1->cols == 1 && CV_ARE_SIZES_EQ(_rvec1, _rvec2) );

   cvConvert( _rvec1, &r1 );
   cvConvert( _rvec2, &r2 );

   cvRodrigues2( &r1, &R1, &dR1dr1 );
   cvRodrigues2( &r2, &R2, &dR2dr2 );

   if( _rvec3 || dr3dr1 || dr3dr1 )
   {
       double _r3[3], _R3[9], _dR3dR1[9*9], _dR3dR2[9*9], _dr3dR3[9*3];
       double _W1[9*3], _W2[3*3];
       CvMat r3 = cvMat(3,1,CV_64F,_r3), R3 = cvMat(3,3,CV_64F,_R3);
       CvMat dR3dR1 = cvMat(9,9,CV_64F,_dR3dR1), dR3dR2 = cvMat(9,9,CV_64F,_dR3dR2);
       CvMat dr3dR3 = cvMat(3,9,CV_64F,_dr3dR3);
       CvMat W1 = cvMat(3,9,CV_64F,_W1), W2 = cvMat(3,3,CV_64F,_W2);

       cvMatMul( &R2, &R1, &R3 );
       cvCalcMatMulDeriv( &R2, &R1, &dR3dR2, &dR3dR1 );

       cvRodrigues2( &R3, &r3, &dr3dR3 );

       if( _rvec3 )
           cvConvert( &r3, _rvec3 );

       if( dr3dr1 )
       {
           cvMatMul( &dr3dR3, &dR3dR1, &W1 );
           cvMatMul( &W1, &dR1dr1, &W2 );
           cvConvert( &W2, dr3dr1 );
       }

       if( dr3dr2 )
       {
           cvMatMul( &dr3dR3, &dR3dR2, &W1 );
           cvMatMul( &W1, &dR2dr2, &W2 );
           cvConvert( &W2, dr3dr2 );
       }
   }

   if( dr3dt1 )
       cvZero( dr3dt1 );
   if( dr3dt2 )
       cvZero( dr3dt2 );

   if( _tvec3 || dt3dr2 || dt3dt1 )
   {
       double _t1[3], _t2[3], _t3[3], _dxdR2[3*9], _dxdt1[3*3], _W3[3*3];
       CvMat t1 = cvMat(3,1,CV_64F,_t1), t2 = cvMat(3,1,CV_64F,_t2);
       CvMat t3 = cvMat(3,1,CV_64F,_t3);
       CvMat dxdR2 = cvMat(3, 9, CV_64F, _dxdR2);
       CvMat dxdt1 = cvMat(3, 3, CV_64F, _dxdt1);
       CvMat W3 = cvMat(3, 3, CV_64F, _W3);

       CV_ASSERT( CV_IS_MAT(_tvec1) && CV_IS_MAT(_tvec2) );
       CV_ASSERT( CV_ARE_SIZES_EQ(_tvec1, _tvec2) && CV_ARE_SIZES_EQ(_tvec1, _rvec1) );

       cvConvert( _tvec1, &t1 );
       cvConvert( _tvec2, &t2 );
       cvMatMulAdd( &R2, &t1, &t2, &t3 );

       if( _tvec3 )
           cvConvert( &t3, _tvec3 );

       if( dt3dr2 || dt3dt1 )
       {
           cvCalcMatMulDeriv( &R2, &t1, &dxdR2, &dxdt1 );
           if( dt3dr2 )
           {
               cvMatMul( &dxdR2, &dR2dr2, &W3 );
               cvConvert( &W3, dt3dr2 );
           }
           if( dt3dt1 )
               cvConvert( &dxdt1, dt3dt1 );
       }
   }

   if( dt3dt2 )
       cvSetIdentity( dt3dt2 );
   if( dt3dr1 )
       cvZero( dt3dr1 );

   __END__;
}


static const int IMG_ACTIVE_THRESH = 5; // minimum number of active points per view to consider it active

// for now it is just a small wrapper for cvCalibrateCamera2, but
// later on it will become more complex thing that will eventually replace cvCalibrateCamera2
bool cvCalibrateCamera2x( const CvMat* _objectPoints,
                       const CvMat* _imagePoints,
                       const CvMat* _npoints,
                       CvMat* _activeImages,
                       CvMat* _activePoints,
                       CvSize imageSize,
                       CvMat* cameraMatrix,
                       CvMat* distCoeffs,
                       int flags=0 )
{
   bool result = false;
   CvMat* objectPoints = (CvMat*)_objectPoints;
   CvMat* imagePoints = (CvMat*)_imagePoints;
   CvMat* npoints = (CvMat*)_npoints;
   CvMat* activeImages = _activeImages;
   CvMat* activePoints = _activePoints;

   CV_FUNCNAME( "cvCalibrateCamera2x" );

   __BEGIN__;

   int i, j, k, nimages, pointsTotal = 0;

   CV_ASSERT( CV_IS_MAT(npoints) && CV_MAT_TYPE(npoints->type) == CV_32SC1 &&
             (npoints->rows == 1 || npoints->cols == 1) );
   nimages = npoints->rows + npoints->cols - 1;
   npoints = cvCreateMat(_npoints->rows, _npoints->cols, _npoints->type);
   cvCopy( _npoints, npoints );

   for( i = 0; i < nimages; i++ )
       pointsTotal += npoints->data.i[i];

   // throw away inactive images/points
   if( activeImages || activePoints )
   {
       int to = 0, from = 0;

       if( activeImages )
       {
           CV_ASSERT( CV_IS_MAT(activeImages) &&
               (CV_MAT_TYPE(activeImages->type) == CV_8UC1 ||
               CV_MAT_TYPE(activeImages->type) == CV_8SC1) &&
               (activeImages->rows == 1 || activeImages->cols == 1) &&
               activeImages->rows + activeImages->cols - 1 == nimages );

           if( !CV_IS_MAT_CONT(activeImages->type) )
           {
               activeImages = cvCreateMat(_activeImages->rows, _activeImages->cols, _activeImages->type);
               cvCopy( _activeImages, activeImages );
           }
       }

       if( activePoints )
       {
           CV_ASSERT( CV_IS_MAT(activePoints) &&
               (CV_MAT_TYPE(activePoints->type) == CV_8UC1 ||
               CV_MAT_TYPE(activePoints->type) == CV_8SC1) &&
               (activePoints->rows == 1 || activePoints->cols == 1) &&
               activePoints->rows + activePoints->cols - 1 == pointsTotal );

           if( !CV_IS_MAT_CONT(activePoints->type) )
           {
               activePoints = cvCreateMat(_activePoints->rows, _activePoints->cols, _activePoints->type);
               cvCopy( _activePoints, activePoints );
           }
       }

       objectPoints = cvCreateMat( _objectPoints->rows, _objectPoints->cols, _objectPoints->type );
       imagePoints = cvCreateMat( _imagePoints->rows, _objectPoints->cols, _imagePoints->type );
       cvCopy( _objectPoints, objectPoints );
       cvCopy( _imagePoints, imagePoints );

       for( i = k = 0; i < nimages; i++ )
       {
           int npt = npoints->data.i[i];
           bool active = !activeImages || activeImages->data.ptr[i] != 0;
           int savedTo = to;
           if( active )
           {
               const uchar* activePt = activePoints ? activePoints->data.ptr + from : 0;
               if( CV_MAT_DEPTH( objectPoints->type ) == CV_32F )
               {
                   const CvPoint2D32f* fromImgPt = (CvPoint2D32f*)imagePoints->data.ptr + from;
                   CvPoint2D32f* toImgPt = (CvPoint2D32f*)imagePoints->data.ptr;
                   const CvPoint3D32f* fromObjPt = (CvPoint3D32f*)objectPoints->data.ptr + from;
                   CvPoint3D32f* toObjPt = (CvPoint3D32f*)objectPoints->data.ptr;

                   for( j = 0; j < npt; j++ )
                   {
                       if( !activePt || activePt[j] )
                       {
                           toImgPt[to] = fromImgPt[j];
                           toObjPt[to++] = fromObjPt[j];
                       }
                   }
               }
               else
               {
                   const CvPoint2D64f* fromImgPt = (CvPoint2D64f*)imagePoints->data.ptr + from;
                   CvPoint2D64f* toImgPt = (CvPoint2D64f*)imagePoints->data.ptr;
                   const CvPoint3D64f* fromObjPt = (CvPoint3D64f*)objectPoints->data.ptr + from;
                   CvPoint3D64f* toObjPt = (CvPoint3D64f*)objectPoints->data.ptr;

                   for( j = 0; j < npt; j++ )
                   {
                       if( !activePt || activePt[j] )
                       {
                           toImgPt[to] = fromImgPt[j];
                           toObjPt[to++] = fromObjPt[j];
                       }
                   }
               }
           }

           if( to - savedTo >= IMG_ACTIVE_THRESH )
               npoints->data.i[k++] = to - savedTo;
           else
               to = savedTo; // discard the image completely
           from += npt;
       }

       if( to == 0 || k < 2 )
           EXIT;
       if( objectPoints->rows == pointsTotal )
           objectPoints->rows = to;
       else
           objectPoints->cols = to;
       if( imagePoints->rows == pointsTotal )
           imagePoints->rows = to;
       else
           imagePoints->cols = to;
       if( npoints->rows == nimages )
           npoints->rows = k;
       else
           npoints->cols = k;
   }

   cvCalibrateCamera2( objectPoints, imagePoints, npoints, imageSize,
                       cameraMatrix, distCoeffs, 0, 0, flags );
   result = true;

   __END__;

   if( objectPoints != _objectPoints )
       cvReleaseMat( &objectPoints );
   if( imagePoints != _imagePoints )
       cvReleaseMat( &imagePoints );
   if( npoints != _npoints )
       cvReleaseMat( &npoints );
   if( activeImages != _activeImages )
       cvReleaseMat( &activeImages );
   if( activePoints != _activePoints )
       cvReleaseMat( &activePoints );
   return result;
}


static int dbCmp( const void* _a, const void* _b )
{
   double a = *(const double*)_a;
   double b = *(const double*)_b;

   return (a > b) - (a < b);
}

#if 0
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
   void doit();
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
#endif

CvLevMarq::CvLevMarq()
{
   prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = 0;
   lambdaLg10 = 0; state = DONE;
   criteria = cvTermCriteria(0,0,0);
   iters = 0;
}

CvLevMarq::CvLevMarq( int nparams, int nerrs, CvTermCriteria criteria0 )
{
   prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = 0;
   init(nparams, nerrs, criteria0);
}

void CvLevMarq::clear()
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

CvLevMarq::~CvLevMarq()
{
   clear();
}

void CvLevMarq::init( int nparams, int nerrs, CvTermCriteria criteria0 )
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

bool CvLevMarq::update( const CvMat*& _param, CvMat*& _J, CvMat*& _err )
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


bool CvLevMarq::updateAlt( const CvMat*& _param, CvMat*& _JtJ, CvMat*& _JtErr, double*& _errNorm )
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
       // jdc debugging
       printf("cvLevMarq done: iter num: %d,  change: %f, errNorm: %f\n", iters, change, errNorm);
       return false;
   }
   // jdc debugging
   printf("cvLevMarq continue: iter num: %d,  change: %le, %le, errNorm: %f\n", 
		   iters, change, criteria.epsilon, errNorm);

   prevErrNorm = errNorm;
   _param = param;
   _JtJ = JtJ;
   _JtErr = JtErr;
   state = CALC_J;
   return true;
}

void CvLevMarq::solve()
{
   const double LOG10 = log(10.);
   double lambda = exp(lambdaLg10*LOG10);

   cvSetIdentity( JtJN, cvRealScalar(lambda) );
   cvAdd( JtJ, JtJN, JtJN );
   cvSVD( JtJN, JtJW, 0, JtJV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
   cvSVBkSb( JtJW, JtJV, JtJV, JtErr, param, CV_SVD_U_T + CV_SVD_V_T );
}


#define CV_CALIB_FIX_INTRINSIC  16

/* Computes optimal rotation matrix (R) and the translation vector (T) from the left
  camera coordinate system to the right camera coordinate system
  (so that if there is point M in the left camera coordinate system,
  it will be R*M+T in the right camera coordinate system).

  Input:
  objectPoints - coordinates of calibration rig feature points in the rig's coordinate system.
  imagePoints1,2 - coordinates of projections of the calibration rig features
                   in the left/right camera view.
  activeImages1,2 - optional input/output 1d vectors (masks) marking the views that should
                    be taken into account by the algorithm. If a pointer is NULL, all the views
                    from the corresponding camera participate in the optimization
                    (for the stereo calibration part itself a view #n participates only
                    if activeImages1[n] != 0 && activeImages2[n] != 0).
                    On output the arrays will contain [the same] final mask, i.e. some images
                    that initially have been marked as active can be excluded during optimization.
                    The opposite never happens.
  activePoints1,2 - optional input/output 1d vectors (masks) marking the individual points
                    in the camera views that should be taken into account by the algorithm.
                    If certain element of activeImages{k} is zero, then all the points
                    in this particular view are ignored. On output each array will contain the final
                    mask, a subset of active points in each view that have been retained by the algorithm.
  cameraMatrix1,2 - the matrices of intrinsic parameters for each camera.
                    If the flag CV_CALIB_FIX_INTRINSIC is set, the matrixes
                    (and the distortion coefficients) will be used as-is,
                    only the optimal [R|t] will be computed. Otherwise, each camera intrinisic
                    parameters will be refined (or computed from scratch) by the stereo
                    calibration optimization algorithm. See also the description of the parameter "flags".
  distCoeffs1,2 - the lens distortion coefficients for each camera.
                  See description of cameraMatrix1,2.
  imageSize - image size in pixels. It is only used if the algorithms is asked to refine/compute
            intrinsic parameters.
  R - output inter-camera rotation matrix.
  T - output inter-camera translation vector.
  E - optional output parameter; the essential matrix.
  F - optional output parameter; the fundamental matrix
  termCrit - termination criteria for the stereo calibration algorithm.
      Either stop after certain number of iterations, or when the computed parameters change by less
      than the threshold.
  imgThreshold - the optimization algorithm deactivates certain image views if the average
      backprojection error for the view exceeds the threshold.
  ptThreshold - the optimization algorithm deactives certain image points if the average backprojection
       error for that point exceeds the threshold.
  flags - if the intrinsic calibration was done prior to calling this function and the
       computed results can be trusted, this parameter should be just set to CV_CALIB_FIX_INTRINSIC.
       Otherwise, the intrinisic parameters for both cameras can be refined by the algorithm
       (if the flag CV_CALIB_USE_INTRINSIC_GUESS) or computed from scratch, in this case
       any combination of flags supported by cvCalibrateCamera2 can be used.
*/
bool cvStereoCalibrate( const CvMat* _objectPoints, const CvMat* _imagePoints1,
                       const CvMat* _imagePoints2, const CvMat* _npoints,
                       CvMat* _activeImages1, CvMat* _activeImages2,
                       CvMat* _activePoints1, CvMat* _activePoints2,
                       CvMat* _cameraMatrix1, CvMat* _distCoeffs1,
                       CvMat* _cameraMatrix2, CvMat* _distCoeffs2,
                       CvSize imageSize, CvMat* _R, CvMat* _T,
                       CvMat* _E, CvMat* _F,
                       /* CvMat* _Rerr, CvMat* _Terr,
                          CvMat* _CM1Err, CvMat* _DC1Err,
                          CvMat* _CM2Err, CvMat* _DC2Err, (???) */
                       CvTermCriteria termCrit, double imgThreshold=50,
                       double ptThreshold=50, int flags=CV_CALIB_FIX_INTRINSIC )
{
   bool result = false;
   CvMat* npoints = 0;
   CvMat* err = 0;
   CvMat* J_LR = 0;
   CvMat* Je = 0;
   CvMat* Ji = 0;
   CvMat* imagePoints[2] = {0,0};
   CvMat* activeImages[2] = {0,0};
   CvMat* activePoints[2] = {0,0};
   CvMat* objectPoints = 0;
   CvMat* RT0 = 0;
   CvLevMarq solver;
   int k;

   CV_FUNCNAME( "cvStereoCalibrate" );

   __BEGIN__;

   signed char* activeMask;
   double a[2][9], dk[2][4], rlr[9];
   CvMat K[2], Dist[2], om_LR, T_LR;
   CvMat R_LR = cvMat(3, 3, CV_64F, rlr);
   int i, j, p, ni = 0, ofs, nimages, nactive, pointsTotal, maxPoints = 0;
   int nparams;
   bool recomputeIntrinsics = false;

   CV_ASSERT( CV_IS_MAT(_imagePoints1) && CV_IS_MAT(_imagePoints2) &&
              CV_IS_MAT(_objectPoints) && CV_IS_MAT(_npoints) &&
              CV_IS_MAT(_R) && CV_IS_MAT(_T) );

   CV_ASSERT( CV_ARE_TYPES_EQ(_imagePoints1, _imagePoints2) &&
              CV_ARE_DEPTHS_EQ(_imagePoints1, _objectPoints) );

   CV_ASSERT( (_npoints->cols == 1 || _npoints->rows == 1) &&
              CV_MAT_TYPE(_npoints->type) == CV_32SC1 );

   nimages = _npoints->cols + _npoints->rows - 1;
   npoints = cvCreateMat( _npoints->rows, _npoints->cols, _npoints->type );
   cvCopy( _npoints, npoints );

   for( i = 0, pointsTotal = 0; i < nimages; i++ )
   {
       maxPoints = MAX(maxPoints, npoints->data.i[i]);
       pointsTotal += npoints->data.i[i];
   }

   objectPoints = cvCreateMat( _objectPoints->rows, _objectPoints->cols,
                               CV_64FC(CV_MAT_CN(_objectPoints->type)));
   cvConvert( _objectPoints, objectPoints );
   cvReshape( objectPoints, objectPoints, 3, 1 );

   for( k = 0; k < 2; k++ )
   {
       const CvMat* points = k == 0 ? _imagePoints1 : _imagePoints2;
       const CvMat* cameraMatrix = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
       const CvMat* distCoeffs = k == 0 ? _distCoeffs1 : _distCoeffs2;
       CvMat* activeImg = k == 0 ? _activeImages1 : _activeImages2;
       CvMat* activePt = k == 0 ? _activePoints1 : _activePoints2;

       int cn = CV_MAT_CN(_imagePoints1->type);
       CV_ASSERT( (CV_MAT_DEPTH(_imagePoints1->type) == CV_32F ||
               CV_MAT_DEPTH(_imagePoints1->type) == CV_64F) &&
              (_imagePoints1->rows == pointsTotal && _imagePoints1->cols*cn == 2 ||
               _imagePoints1->rows == 1 && _imagePoints1->cols == pointsTotal && cn == 2) );

       K[k] = cvMat(3,3,CV_64F,a[k]);
       Dist[k] = cvMat(1,4,CV_64F,dk[k]);

       imagePoints[k] = cvCreateMat( points->rows, points->cols, CV_64FC(CV_MAT_CN(points->type)));
       cvConvert( points, imagePoints[k] );
       cvReshape( imagePoints[k], imagePoints[k], 2, 1 );

       if( activePt )
       {
           CV_ASSERT( CV_IS_MAT(activePt) &&
               (CV_MAT_TYPE(activePt->type) == CV_8UC1 || CV_MAT_TYPE(activePt->type) == CV_8SC1) &&
               (activePt->rows == 1 || activePt->cols == 1) &&
               activePt->rows + activePt->cols - 1 == pointsTotal );
           activePoints[k] = cvCreateMat( activePt->rows, activePt->cols, activePt->type );
           cvCopy( activePt, activePoints[k] );
           cvReshape( activePt, activePt, 1, 1 );
       }
       else
       {
           activePoints[k] = cvCreateMat( 1, pointsTotal, CV_8UC1 );
           cvSet( activePoints[k], cvScalarAll(1) );
       }

       if( activeImg )
       {
           CV_ASSERT( CV_IS_MAT(activeImg) &&
               (CV_MAT_TYPE(activeImg->type) == CV_8UC1 || CV_MAT_TYPE(activeImg->type) == CV_8SC1) &&
               (activeImg->rows == 1 || activeImg->cols == 1) &&
               activeImg->rows + activeImg->cols - 1 == nimages );
           activeImages[k] = cvCreateMat( activeImg->rows, activeImg->cols, activeImg->type );
           cvCopy( activeImg, activeImages[k] );
       }
       else
       {
           activeImages[k] = cvCreateMat( 1, nimages, CV_8UC1 );
           cvSet( activeImages[k], cvScalarAll(1) );
       }

       if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS|CV_CALIB_FIX_ASPECT_RATIO) )
           cvConvert( cameraMatrix, &K[k] );

       if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS) )
           cvConvert( distCoeffs, &Dist[k] );
       else
       {
           result = cvCalibrateCamera2x( objectPoints, imagePoints[k],
               npoints, activeImages[k], activePoints[k], imageSize,
               &K[k], &Dist[k], flags );
           if( !result )
               EXIT;
       }
   }

   recomputeIntrinsics = (flags & CV_CALIB_FIX_INTRINSIC) == 0;

   err = cvCreateMat( maxPoints*2, 1, CV_64F );
   Je = cvCreateMat( maxPoints*2, 6, CV_64F );
   J_LR = cvCreateMat( maxPoints*2, 6, CV_64F );
   Ji = cvCreateMat( maxPoints*2, 8, CV_64F );
   cvZero( Ji );

   // we optimize for the inter-camera R(3),t(3), then, optionally,
   // for intrinisic parameters of each camera ((fx,fy,cx,cy,k1,k2,p1,p2) ~ 8 parameters).
   nparams = 6 + (recomputeIntrinsics ? 8*2 : 0);

   for( i = nactive = ofs = 0; i < nimages; ofs += ni, i++ )
   {
       int activeLeft = 0, activeRight = 0;
       ni = npoints->data.i[i];
       if( activeImages[0]->data.ptr[i] == 0 || activeImages[1]->data.ptr[i] == 0 )
       {
           activeImages[0]->data.ptr[i] = 0;
           continue;
       }
       activeImages[0]->data.ptr[i] = 1;

       for( j = 0; j < ni; j++ )
       {
           activeLeft += activePoints[0]->data.ptr[j+ofs];
           activeRight += activePoints[1]->data.ptr[j+ofs];
       }
       if( activeLeft < IMG_ACTIVE_THRESH || activeRight < IMG_ACTIVE_THRESH )
       {
           activeImages[0]->data.ptr[i] = 0;
           continue;
       }
       nactive++;
       nparams += 6; // ... and for R{left}, T{left} for each pair of active views
   }

   activeMask = (signed char*)activeImages[0]->data.ptr;

   // storage for initial [om(R){i}|t{i}] (in order to compute the median for each component)
   RT0 = cvCreateMat( 6, nactive, CV_64F );

   solver.init( nparams, 0, termCrit );

   /*
      1. Compute initial estimate of pose

      For each image, compute:
         R(om) is the rotation matrix of om
         om(R) is the rotation vector of R
         R_ref = R(om_right) * R(om_left)'
         T_ref_list = [T_ref_list; T_right - R_ref * T_left]
         om_ref_list = {om_ref_list; om(R_ref)]

      om = median(om_ref_list)
      T = median(T_ref_list)
   */

   for( i = j = ofs = 0; i < nimages; ofs += ni, i++ )
   {
       ni = npoints->data.i[i];
       CvMat objpt_i;
       double _om[2][3], r[2][9], t[2][3];
       CvMat om[2], R[2], T[2], imgpt_i[2];

       if( activeMask[i] <= 0 )
       {
           j += activeMask[i] < 0;
           continue;
       }

       objpt_i = cvMat(1, ni, CV_64FC3, objectPoints->data.db + ofs*3);
       for( k = 0; k < 2; k++ )
       {
           imgpt_i[k] = cvMat(1, ni, CV_64FC2, imagePoints[k]->data.db + ofs*2);
           om[k] = cvMat(3, 1, CV_64F, _om[k]);
           R[k] = cvMat(3, 3, CV_64F, r[k]);
           T[k] = cvMat(3, 1, CV_64F, t[k]);

           // FIXME: here we ignore activePoints[k] because of
           // the limited API of cvFindExtrnisicCameraParams2
           cvFindExtrinsicCameraParams2( &objpt_i, &imgpt_i[k], &K[k], &Dist[k], &om[k], &T[k] );
           cvRodrigues2( &om[k], &R[k] );
           if( k == 0 )
           {
               // save initial om_left and T_left
               solver.param->data.db[(j+1)*6] = _om[0][0];
               solver.param->data.db[(j+1)*6 + 1] = _om[0][1];
               solver.param->data.db[(j+1)*6 + 2] = _om[0][2];
               solver.param->data.db[(j+1)*6 + 3] = t[0][0];
               solver.param->data.db[(j+1)*6 + 4] = t[0][1];
               solver.param->data.db[(j+1)*6 + 5] = t[0][2];
           }
       }
       cvGEMM( &R[1], &R[0], 1, 0, 0, &R[0], CV_GEMM_B_T );
       cvGEMM( &R[0], &T[0], -1, &T[1], 1, &T[1] );
       cvRodrigues2( &R[0], &T[0] );
       RT0->data.db[j] = t[0][0];
       RT0->data.db[j + nactive] = t[0][1];
       RT0->data.db[j + nactive*2] = t[0][2];
       RT0->data.db[j + nactive*3] = t[1][0];
       RT0->data.db[j + nactive*4] = t[1][1];
       RT0->data.db[j + nactive*5] = t[1][2];
       j++;
   }

   // find the medians and save the first 6 parameters
   for( i = 0; i < 6; i++ )
   {
       qsort( RT0->data.db + i*nactive, nactive, CV_ELEM_SIZE(RT0->type), dbCmp );
       solver.param->data.db[i] = nactive % 2 != 0 ? RT0->data.db[i*nactive + nactive/2] :
           (RT0->data.db[i*nactive + nactive/2 - 1] + RT0->data.db[i*nactive + nactive/2])*0.5;
   }

   if( recomputeIntrinsics )
       for( k = 0; k < 2; k++ )
       {
           double* iparam = solver.param->data.db + (nactive+1)*6 + k*8;
           iparam[0] = a[k][0];
           iparam[1] = a[k][4];
           iparam[2] = a[k][2];
           iparam[3] = a[k][5];
           iparam[4] = dk[k][0];
           iparam[5] = dk[k][1];
           iparam[6] = dk[k][2];
           iparam[7] = dk[k][3];
       }

   om_LR = cvMat(3, 1, CV_64F, solver.param->data.db);
   T_LR = cvMat(3, 1, CV_64F, solver.param->data.db + 3);

   for(;;)
   {
       const CvMat* param = 0;
       CvMat tmpImgPoints;
       CvMat* JtJ = 0;
       CvMat* JtErr = 0;
       double* errNorm = 0;
       double _omR[3], tR[3];
       double _dr3dr1[9], _dr3dr2[9], /*_dt3dr1[9],*/ _dt3dr2[9], _dt3dt1[9], _dt3dt2[9];
       CvMat dr3dr1 = cvMat(3, 3, CV_64F, _dr3dr1);
       CvMat dr3dr2 = cvMat(3, 3, CV_64F, _dr3dr2);
       //CvMat dt3dr1 = cvMat(3, 3, CV_64F, _dt3dr1);
       CvMat dt3dr2 = cvMat(3, 3, CV_64F, _dt3dr2);
       CvMat dt3dt1 = cvMat(3, 3, CV_64F, _dt3dt1);
       CvMat dt3dt2 = cvMat(3, 3, CV_64F, _dt3dt2);
       CvMat om[2], T[2], imgpt_i[2];
       CvMat dpdrot_hdr, dpdt_hdr, dpdf_hdr, dpdc_hdr, dpdk_hdr;
       CvMat *dpdrot = &dpdrot_hdr, *dpdt = &dpdt_hdr, *dpdf = 0, *dpdc = 0, *dpdk = 0;

       if( !solver.updateAlt( param, JtJ, JtErr, errNorm ))
           break;

       cvRodrigues2( &om_LR, &R_LR );
       if( JtJ )
           cvZero( JtJ );
       if( JtErr )
           cvZero( JtErr );
       if( errNorm )
           *errNorm = 0;

       om[1] = cvMat(3,1,CV_64F,_omR);
       T[1] = cvMat(3,1,CV_64F,tR);

       if( recomputeIntrinsics )
       {
           dpdf = &dpdf_hdr;
           dpdc = &dpdc_hdr;
           dpdk = &dpdk_hdr;
           for( k = 0; k < 2; k++ )
           {
               double* iparam = solver.param->data.db + (nactive+1)*6 + k*8;
               a[k][0] = iparam[0];
               a[k][4] = iparam[1];
               a[k][2] = iparam[2];
               a[k][5] = iparam[3];
               dk[k][0] = iparam[4];
               dk[k][1] = iparam[5];
               dk[k][2] = iparam[6];
               dk[k][3] = iparam[7];
           }
       }

       for( i = j = ofs = 0; i < nimages; ofs += ni, i++ )
       {
           ni = npoints->data.i[i];
           CvMat objpt_i, _part;

           // activeMask[i]:
           //    =0 - (was initially deactivated (in the left and/or the right view) by user
           //    <0 - has been deactivated by the optimization algorithm
           //    >0 - still active
           if( activeMask[i] <= 0 )
           {
               j += activeMask[i] < 0;
               continue;
           }

           om[0] = cvMat(3,1,CV_64F,solver.param->data.db+(j+1)*6);
           T[0] = cvMat(3,1,CV_64F,solver.param->data.db+(j+1)*6+3);

           if( JtJ || JtErr )
               cvComposeRT( &om[0], &T[0], &om_LR, &T_LR, &om[1], &T[1], &dr3dr1, 0,
                            &dr3dr2, 0, 0, &dt3dt1, &dt3dr2, &dt3dt2 );
           else
               cvComposeRT( &om[0], &T[0], &om_LR, &T_LR, &om[1], &T[1] );

           objpt_i = cvMat(1, ni, CV_64FC3, objectPoints->data.db + ofs*3);
           err->rows = Je->rows = J_LR->rows = Ji->rows = ni*2;
           cvReshape( err, &tmpImgPoints, 2, 1 );

           cvGetCols( Ji, &dpdf_hdr, 0, 2 );
           cvGetCols( Ji, &dpdc_hdr, 2, 4 );
           cvGetCols( Ji, &dpdk_hdr, 4, 8 );
           cvGetCols( Je, &dpdrot_hdr, 0, 3 );
           cvGetCols( Je, &dpdt_hdr, 3, 6 );

           for( k = 0; k < 2; k++ )
           {
               double maxErr, l2err;
               imgpt_i[k] = cvMat(1, ni, CV_64FC2, imagePoints[k]->data.db + ofs*2);

               if( JtJ || JtErr )
                   cvProjectPoints2( &objpt_i, &om[k], &T[k], &K[k], &Dist[k],
                           &tmpImgPoints, dpdrot, dpdt, dpdf, dpdc, dpdk );
               else
                   cvProjectPoints2( &objpt_i, &om[k], &T[k], &K[k], &Dist[k], &tmpImgPoints );
               cvSub( &tmpImgPoints, &imgpt_i[k], &tmpImgPoints );

               l2err = cvNorm( &tmpImgPoints, 0, CV_L2 );
               maxErr = cvNorm( &tmpImgPoints, 0, CV_C );

               // TODO: once again, we ignore activePoints[k] here.
               if( JtJ || JtErr )
               {
                   int iofs = (nactive+1)*6 + k*8, eofs = (j+1)*6;
                   assert( JtJ && JtErr );

                   if( k == 1 )
                   {
                       // d(err_{x|y}R) ~ de3
                       // convert de3/{dr3,dt3} => de3{dr1,dt1} & de3{dr2,dt2}
                       for( p = 0; p < ni*2; p++ )
                       {
                           CvMat de3dr3 = cvMat( 1, 3, CV_64F, Je->data.ptr + Je->step*p );
                           CvMat de3dt3 = cvMat( 1, 3, CV_64F, de3dr3.data.db + 3 );
                           CvMat de3dr2 = cvMat( 1, 3, CV_64F, J_LR->data.ptr + J_LR->step*p );
                           CvMat de3dt2 = cvMat( 1, 3, CV_64F, de3dr2.data.db + 3 );
                           double _de3dr1[3], _de3dt1[3];
                           CvMat de3dr1 = cvMat( 1, 3, CV_64F, _de3dr1 );
                           CvMat de3dt1 = cvMat( 1, 3, CV_64F, _de3dt1 );

                           cvMatMul( &de3dr3, &dr3dr1, &de3dr1 );
                           cvMatMul( &de3dt3, &dt3dt1, &de3dt1 );

                           cvMatMul( &de3dr3, &dr3dr2, &de3dr2 );
                           cvMatMulAdd( &de3dt3, &dt3dr2, &de3dr2, &de3dr2 );

                           cvMatMul( &de3dt3, &dt3dt2, &de3dt2 );

                           cvCopy( &de3dr1, &de3dr3 );
                           cvCopy( &de3dt1, &de3dt3 );
                       }

                       cvGetSubRect( JtJ, &_part, cvRect(0, 0, 6, 6) );
                       cvGEMM( J_LR, J_LR, 1, &_part, 1, &_part, CV_GEMM_A_T );

                       cvGetSubRect( JtJ, &_part, cvRect(eofs, 0, 6, 6) );
                       cvGEMM( J_LR, Je, 1, 0, 0, &_part, CV_GEMM_A_T );

                       cvGetRows( JtErr, &_part, 0, 6 );
                       cvGEMM( J_LR, err, 1, &_part, 1, &_part, CV_GEMM_A_T );
                   }

                   cvGetSubRect( JtJ, &_part, cvRect(eofs, eofs, 6, 6) );
                   cvGEMM( Je, Je, 1, &_part, 1, &_part, CV_GEMM_A_T );

                   cvGetRows( JtErr, &_part, eofs, eofs + 6 );
                   cvGEMM( Je, err, 1, &_part, 1, &_part, CV_GEMM_A_T );

                   if( recomputeIntrinsics )
                   {
                       cvGetSubRect( JtJ, &_part, cvRect(iofs, iofs, 8, 8) );
                       cvGEMM( Ji, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
                       cvGetSubRect( JtJ, &_part, cvRect(iofs, eofs, 8, 6) );
                       cvGEMM( Je, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
                       if( k == 1 )
                       {
                           cvGetSubRect( JtJ, &_part, cvRect(iofs, 0, 8, 6) );
                           cvGEMM( J_LR, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
                       }
                       cvGetRows( JtErr, &_part, iofs, iofs + 8 );
                       cvGEMM( Ji, err, 1, &_part, 1, &_part, CV_GEMM_A_T );
                   }
               }

               if( maxErr > imgThreshold )
               {
                   activeMask[i] = (signed char)-1;
                   break;
               }

               if( errNorm )
                   *errNorm += l2err*l2err;
           }
           j++;
       }
       if( errNorm && solver.iters % 10 == 0 )
       {
           printf("%d. |err|=%g\n", solver.iters, *errNorm );
       }
   }

   cvRodrigues2( &om_LR, &R_LR );
   if( _R->rows == 1 || _R->cols == 1 )
       cvConvert( &om_LR, _R );
   else
       cvConvert( &R_LR, _R );
   cvConvert( &T_LR, _T );

   if( recomputeIntrinsics )
   {
       cvConvert( &K[0], _cameraMatrix1 );
       cvConvert( &K[1], _cameraMatrix2 );
       cvConvert( &Dist[0], _distCoeffs1 );
       cvConvert( &Dist[1], _distCoeffs2 );
   }

   if( _activeImages1 )
       cvCopy( activeImages[0], _activeImages1 );
   if( _activeImages2 )
       cvCopy( activeImages[1], _activeImages2 );

   if( _E || _F )
   {
       double* t = T_LR.data.db;
       double tx[] =
       {
           0, -t[2], t[1],
           t[2], 0, -t[0],
           -t[1], t[0], 0
       };
       CvMat Tx = cvMat(3, 3, CV_64F, tx);
       double e[9], f[9];
       CvMat E = cvMat(3, 3, CV_64F, e);
       CvMat F = cvMat(3, 3, CV_64F, f);
       cvMatMul( &Tx, &R_LR, &E );
       if( _E )
           cvConvert( &E, _E );
       if( _F )
       {
           double ik[9];
           CvMat iK = cvMat(3, 3, CV_64F, ik);
           cvInvert(&K[1], &iK);
           cvGEMM( &iK, &E, 1, 0, 0, &E, CV_GEMM_A_T );
           cvInvert(&K[0], &iK);
           cvMatMul(&E, &iK, &F);
           cvConvertScale( &F, _F, fabs(f[8]) > 0 ? 1./f[8] : 1 );
       }
   }

   __END__;

   cvReleaseMat( &npoints );
   cvReleaseMat( &err );
   cvReleaseMat( &J_LR );
   cvReleaseMat( &Je );
   cvReleaseMat( &Ji );
   cvReleaseMat( &objectPoints );
   cvReleaseMat( &RT0 );

   for( k = 0; k < 2; k++ )
   {
       cvReleaseMat( &imagePoints[k] );
       cvReleaseMat( &activeImages[k] );
       cvReleaseMat( &activePoints[k] );
   };

   return result;
}


int cvStereoCalcHomographiesFromF(
   CvSize imgSize, const CvMat* _points1, const CvMat* _points2,
   const CvMat* F0, CvMat* _H1, CvMat* _H2, double threshold )
{
   int result = 0;
   CvMat* _m1 = 0;
   CvMat* _m2 = 0;
   CvMat* _lines1 = 0;
   CvMat* _lines2 = 0;

   CV_FUNCNAME( "cvStereoCalcHomographiesFromF" );

   __BEGIN__;

   int i, j, npoints;
   double cx, cy;
   double u[9], v[9], w[9], f[9], h1[9], h2[9], h0[9], e2[3];
   CvMat E2 = cvMat( 3, 1, CV_64F, e2 );
   CvMat U = cvMat( 3, 3, CV_64F, u );
   CvMat V = cvMat( 3, 3, CV_64F, v );
   CvMat W = cvMat( 3, 3, CV_64F, w );
   CvMat F = cvMat( 3, 3, CV_64F, f );
   CvMat H1 = cvMat( 3, 3, CV_64F, h1 );
   CvMat H2 = cvMat( 3, 3, CV_64F, h2 );
   CvMat H0 = cvMat( 3, 3, CV_64F, h0 );

   CvPoint2D64f* m1;
   CvPoint2D64f* m2;
   CvPoint3D64f* lines1;
   CvPoint3D64f* lines2;

   CV_ASSERT( CV_IS_MAT(_points1) && CV_IS_MAT(_points2) &&
       (_points1->rows == 1 || _points1->cols == 1) &&
       (_points2->rows == 1 || _points2->cols == 1) &&
       CV_ARE_SIZES_EQ(_points1, _points2) );

   npoints = _points1->rows * _points1->cols * CV_MAT_CN(_points1->type) / 2;

   _m1 = cvCreateMat( _points1->rows, _points1->cols, CV_64FC(CV_MAT_CN(_points1->type)) );
   _m2 = cvCreateMat( _points2->rows, _points2->cols, CV_64FC(CV_MAT_CN(_points2->type)) );
   _lines1 = cvCreateMat( 1, npoints, CV_64FC3 );
   _lines2 = cvCreateMat( 1, npoints, CV_64FC3 );

   cvConvert( F0, &F );

   cvSVD( (CvMat*)&F, &W, &U, &V, CV_SVD_U_T + CV_SVD_V_T );
   W.data.db[8] = 0.;
   cvGEMM( &U, &W, 1, 0, 0, &W, CV_GEMM_A_T );
   cvMatMul( &W, &V, &F );

   cx = cvRound( imgSize.width*0.5 );
   cy = cvRound( imgSize.height*0.5 );

   cvZero( _H1 );
   cvZero( _H2 );

   cvConvert( _points1, _m1 );
   cvConvert( _points2, _m2 );
   cvReshape( _m1, _m1, 2, 1 );
   cvReshape( _m1, _m1, 2, 1 );

   m1 = (CvPoint2D64f*)_m1->data.ptr;
   m2 = (CvPoint2D64f*)_m2->data.ptr;
   lines1 = (CvPoint3D64f*)_lines1->data.ptr;
   lines2 = (CvPoint3D64f*)_lines2->data.ptr;

   cvComputeCorrespondEpilines( _m1, 1, &F, _lines1 );
   cvComputeCorrespondEpilines( _m2, 2, &F, _lines2 );

   // measure distance from points to the corresponding epilines, mark outliers
   for( i = j = 0; i < npoints; i++ )
   {
       if( fabs(m1[i].x*lines2[i].x +
                m1[i].y*lines2[i].y +
                lines2[i].z) <= threshold &&
           fabs(m2[i].x*lines1[i].x +
                m2[i].y*lines1[i].y +
                lines1[i].z) <= threshold )
       {
           if( j > i )
           {
               m1[j] = m1[i];
               m2[j] = m2[i];
           }
           j++;
       }
   }

   npoints = j;
   if( npoints == 0 )
       EXIT;

   {
   _m1->cols = _m2->cols = npoints;
   memcpy( E2.data.db, U.data.db + 6, sizeof(e2));
   cvScale( &E2, &E2, e2[2] > 0 ? 1 : -1 );

   double t[] =
   {
       1, 0, -cx,
       0, 1, -cy,
       0, 0, 1
   };
   CvMat T = cvMat(3, 3, CV_64F, t);
   cvMatMul( &T, &E2, &E2 );

   int mirror = e2[0] < 0;
   double d = MAX(sqrt(e2[0]*e2[0] + e2[1]*e2[1]),DBL_EPSILON);
   double alpha = e2[0]/d;
   double beta = e2[1]/d;
   double r[] =
   {
       alpha, beta, 0,
       -beta, alpha, 0,
       0, 0, 1
   };
   CvMat R = cvMat(3, 3, CV_64F, r);
   cvMatMul( &R, &T, &T );
   cvMatMul( &R, &E2, &E2 );
   double invf = fabs(e2[2]) < 1e-6*fabs(e2[0]) ? 0 : -e2[2]/e2[0];
   double k[] =
   {
       1, 0, 0,
       0, 1, 0,
       invf, 0, 1
   };
   CvMat K = cvMat(3, 3, CV_64F, k);
   cvMatMul( &K, &T, &H2 );
   cvMatMul( &K, &E2, &E2 );

   double it[] =
   {
       1, 0, cx,
       0, 1, cy,
       0, 0, 1
   };
   CvMat iT = cvMat( 3, 3, CV_64F, it );
   cvMatMul( &iT, &H2, &H2 );

   memcpy( E2.data.db, U.data.db + 6, sizeof(e2));
   cvScale( &E2, &E2, e2[2] > 0 ? 1 : -1 );

   double e2_x[] =
   {
       0, -e2[2], e2[1],
      e2[2], 0, -e2[0],
      -e2[1], e2[0], 0
   };
   double e2_111[] =
   {
       e2[0], e2[0], e2[0],
       e2[1], e2[1], e2[1],
       e2[2], e2[2], e2[2],
   };
   CvMat E2_x = cvMat(3, 3, CV_64F, e2_x);
   CvMat E2_111 = cvMat(3, 3, CV_64F, e2_111);
   cvMatMulAdd(&E2_x, &F, &E2_111, &H0 );
   cvMatMul(&H2, &H0, &H0);
   CvMat E1=cvMat(3, 1, CV_64F, V.data.db+6);
   cvMatMul(&H0, &E1, &E1);

   cvPerspectiveTransform( _m1, _m1, &H0 );
   cvPerspectiveTransform( _m2, _m2, &H2 );
   CvMat A = cvMat( 1, npoints, CV_64FC3, lines1 ), BxBy, B;
   double a[9], atb[3], x[3];
   CvMat AtA = cvMat( 3, 3, CV_64F, a );
   CvMat AtB = cvMat( 3, 1, CV_64F, atb );
   CvMat X = cvMat( 3, 1, CV_64F, x );
   cvConvertPointsHomogenious( _m1, &A );
   cvReshape( &A, &A, 1, npoints );
   cvReshape( _m2, &BxBy, 1, npoints );
   cvGetCol( &BxBy, &B, 0 );
   cvGEMM( &A, &A, 1, 0, 0, &AtA, CV_GEMM_A_T );
   cvGEMM( &A, &B, 1, 0, 0, &AtB, CV_GEMM_A_T );
   cvSolve( &AtA, &AtB, &X, CV_SVD_SYM );

   double ha[] =
   {
       x[0], x[1], x[2],
       0, 1, 0,
       0, 0, 1
   };
   CvMat Ha = cvMat(3, 3, CV_64F, ha);
   cvMatMul( &Ha, &H0, &H1 );
   cvPerspectiveTransform( _m1, _m1, &Ha );

   if( mirror )
   {
       double mm[] = { -1, 0, cx*2, 0, -1, cy*2, 0, 0, 1 };
       CvMat MM = cvMat(3, 3, CV_64F, mm);
       cvMatMul( &MM, &H1, &H1 );
       cvMatMul( &MM, &H2, &H2 );
   }

   cvConvert( &H1, _H1 );
   cvConvert( &H2, _H2 );

   result = 1;
   }

   __END__;

   cvReleaseMat( &_m1 );
   cvReleaseMat( &_m2 );
   cvReleaseMat( &_lines1 );
   cvReleaseMat( &_lines2 );

   return result;
}


void cvStereoInitUndistortRectifyMap(
   const CvMat* _K, const CvMat* _D,
   const CvMat* _H, CvArr* mapxarr, CvArr* mapyarr )
{
   CV_FUNCNAME( "cvStereoInitUndistortRectifyMap" );

   __BEGIN__;

   double k[9], d[4], h[9], ih[9];
   int coi1 = 0, coi2 = 0;
   CvMat mapxstub, *_mapx = (CvMat*)mapxarr;
   CvMat mapystub, *_mapy = (CvMat*)mapyarr;
   CvMat K = cvMat( 3, 3, CV_64F, k ), D;
   CvMat H = cvMat( 3, 3, CV_64F, h ), iH = cvMat( 3, 3, CV_64F, ih );
   int i, j;
   double u0, v0, x0, y0, fx, fy, _fx, _fy, k1, k2, p1, p2;
   CvSize size;

   CV_CALL( _mapx = cvGetMat( _mapx, &mapxstub, &coi1 ));
   CV_CALL( _mapy = cvGetMat( _mapy, &mapystub, &coi2 ));

   CV_ASSERT( coi1 == 0 && coi2 == 0 ); // The function does not support COI
   CV_ASSERT( CV_MAT_TYPE(_mapx->type) == CV_32FC1 );
   CV_ASSERT( CV_ARE_TYPES_EQ(_mapx, _mapy ) && CV_ARE_SIZES_EQ( _mapx, _mapy ));
   CV_ASSERT( CV_IS_MAT(_H) && _H->rows == 3 && _H->cols == 3 );
   cvConvert( _H, &H );
   cvInvert( &H, &iH );

   if( _K )
   {
       CV_ASSERT( CV_IS_MAT(_K) && _K->rows == 3 && _K->cols == 3 );
       cvConvert( _K, &K );
   }
   else
       cvSetIdentity( &K );

   if( _D )
   {
       CV_ASSERT( CV_IS_MAT(_D) && (_D->rows == 1 || _D->cols == 1) && _D->rows + _D->cols - 1 == 4 );
       D = cvMat( _D->rows, _D->cols, CV_64F, d );
       cvConvert( _D, &D );
   }
   else
   {
       D = cvMat( 1, 4, CV_64F, d );
       cvZero( &D );
   }

   size = cvGetMatSize(_mapx);

   u0 = k[2]; v0 = k[5];
   x0 = size.width*0.5;
   y0 = size.height*0.5;
   fx = k[0]; fy = k[4];
   _fx = 1./fx; _fy = 1./fy;
   k1 = d[0]; k2 = d[1];
   p1 = d[2]; p2 = d[3];

   for( i = 0; i < size.height; i++ )
   {
       float* mapx = (float*)(_mapx->data.ptr + _mapx->step*i);
       float* mapy = (float*)(_mapy->data.ptr + _mapy->step*i);
       double _x = i*ih[1] + ih[2];
       double _y = i*ih[4] + ih[5];
       double _w = i*ih[7] + ih[8];

       for( j = 0; j < size.width; j++, _x += ih[0], _y += ih[3], _w += ih[6] )
       {
           double iw = 1./_w;
           double x = _fx*(_x*iw - x0);
           double y = _fy*(_y*iw - y0);
           double x2 = x*x, y2 = y*y;
           double r2 = x2 + y2, r4 = r2*r2, _2xy = 2*x*y;
           double kr = 1 + k1*r2 + k2*r4;
           double u = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
           double v = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;
           mapx[j] = (float)u;
           mapy[j] = (float)v;
       }
   }

   __END__;
}


static void
cvUndistortPoints( CvSize imageSize, const CvMat* _src, CvMat* _dst, const CvMat* _K, const CvMat* _D )
{
   CV_FUNCNAME( "cvUndistortPoints" );

   __BEGIN__;

   double K[3][3], D[4], fx, fy, ifx, ify, cx, cy;
   CvMat _Kd=cvMat(3, 3, CV_64F, K), _Dd;
   const CvPoint2D32f* srcf;
   const CvPoint2D64f* srcd;
   CvPoint2D32f* dstf;
   CvPoint2D64f* dstd;
   int sstep, dstep;
   int i, j, n;
   int type;

   CV_ASSERT( CV_IS_MAT(_src) && CV_IS_MAT(_dst) &&
       (_src->rows == 1 || _src->cols == 1) &&
       (_dst->rows == 1 || _dst->cols == 1) &&
       CV_ARE_SIZES_EQ(_src, _dst) && CV_ARE_TYPES_EQ(_src, _dst) &&
       (CV_MAT_TYPE(_src->type) == CV_32FC2 || CV_MAT_TYPE(_src->type) == CV_64FC2));

   CV_ASSERT( CV_IS_MAT(_K) && CV_IS_MAT(_D) &&
       _K->rows == 3 && _K->cols == 3 &&
       (_D->rows == 1 || _D->cols == 1) && _D->rows + _D->cols - 1 == 4 );
   _Dd = cvMat( _D->rows, _D->cols, CV_64F, D);
   cvConvert( _K, &_Kd );
   cvConvert( _D, &_Dd );

   srcf = (const CvPoint2D32f*)_src->data.ptr;
   srcd = (const CvPoint2D64f*)_src->data.ptr;
   dstf = (CvPoint2D32f*)_src->data.ptr;
   dstd = (CvPoint2D64f*)_src->data.ptr;
   sstep = _src->rows == 1 ? 1 : _src->step/CV_ELEM_SIZE(_src->type);
   dstep = _dst->rows == 1 ? 1 : _dst->step/CV_ELEM_SIZE(_dst->type);

   type = CV_MAT_TYPE(_src->type);
   n = _src->rows + _src->cols - 1;

   fx = K[0][0];
   fy = K[1][1];
   ifx = 1./fx;
   ify = 1./fy;
   cx = K[0][2];
   cy = K[1][2];

   for( i = 0; i < n; i++ )
   {
       double x, y, x0, y0;
       if( type == CV_32FC2 )
       {
           x = srcf[i*sstep].x;
           y = srcf[i*sstep].y;
       }
       else
       {
           x = srcd[i*sstep].x;
           y = srcd[i*sstep].y;
       }

       x0 = x = (x - cx)*ifx;
       y0 = y = (y - cy)*ify;

       // compensate distortion iteratively
       for( j = 0; j < 5; j++ )
       {
           double r2 = x*x + y*y;
           double icdist = 1./(1 + D[0]*r2 + D[1]*r2*r2);
           double deltaX = 2*D[2]*x*y + D[3]*(r2 + 2*x*x);
           double deltaY = D[2]*(r2 + 2*y*y) + 2*D[3]*x*y;
           x = (x0 - deltaX)*icdist;
           y = (y0 - deltaY)*icdist;
       }

       x = x*fx + imageSize.width*0.5;
       y = y*fy + imageSize.height*0.5;

       if( type == CV_32FC2 )
       {
           dstf[i*dstep].x = (float)x;
           dstf[i*dstep].y = (float)y;
       }
       else
       {
           dstd[i*dstep].x = x;
           dstd[i*dstep].y = y;
       }
   }

   __END__;
}


static void
testComposeRT()
{
   double eps = 1e-2;
   int iter, maxIter = 10;
   CvRNG rng = cvRNG(-1);

   for( iter = 0; iter < maxIter; iter++ )
   {
       double om1[3], om2[3], om3[3], om3r[3], om3p[3], T1[3], T2[3], T3[3], T3r[3], T3p[3];
       double dom1[3], dom2[3], dT1[3], dT2[3];
       double dom3dom1[9], dom3dom2[9], dom3dT1[9], dom3dT2[9];
       double dT3dom1[9], dT3dom2[9], dT3dT1[9], dT3dT2[9];

       CvMat _om1 = cvMat(3, 1, CV_64F, om1);
       CvMat _om2 = cvMat(3, 1, CV_64F, om2);
       CvMat _om3 = cvMat(3, 1, CV_64F, om3);
       CvMat _om3r = cvMat(3, 1, CV_64F, om3r);
       CvMat _om3p = cvMat(3, 1, CV_64F, om3p);
       CvMat _T1 = cvMat(3, 1, CV_64F, T1);
       CvMat _T2 = cvMat(3, 1, CV_64F, T2);
       CvMat _T3 = cvMat(3, 1, CV_64F, T3);
       CvMat _T3r = cvMat(3, 1, CV_64F, T3r);
       CvMat _T3p = cvMat(3, 1, CV_64F, T3p);

       CvMat _dom1 = cvMat(3, 1, CV_64F, dom1);
       CvMat _dom2 = cvMat(3, 1, CV_64F, dom2);
       CvMat _dT1 = cvMat(3, 1, CV_64F, dT1);
       CvMat _dT2 = cvMat(3, 1, CV_64F, dT2);

       CvMat _dom3dom1 = cvMat(3, 3, CV_64F, dom3dom1);
       CvMat _dom3dom2 = cvMat(3, 3, CV_64F, dom3dom2);
       CvMat _dom3dT1 = cvMat(3, 3, CV_64F, dom3dT1);
       CvMat _dom3dT2 = cvMat(3, 3, CV_64F, dom3dT2);
       CvMat _dT3dom1 = cvMat(3, 3, CV_64F, dT3dom1);
       CvMat _dT3dom2 = cvMat(3, 3, CV_64F, dT3dom2);
       CvMat _dT3dT1 = cvMat(3, 3, CV_64F, dT3dT1);
       CvMat _dT3dT2 = cvMat(3, 3, CV_64F, dT3dT2);

       cvRandArr(&rng, &_om1, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
       cvRandArr(&rng, &_om2, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
       cvRandArr(&rng, &_T1, CV_RAND_NORMAL, cvScalar(0), cvScalar(10));
       cvRandArr(&rng, &_T2, CV_RAND_NORMAL, cvScalar(0), cvScalar(10));

       cvRandArr(&rng, &_dom1, CV_RAND_NORMAL, cvScalar(0), cvScalar(1e-3));
       cvRandArr(&rng, &_dom2, CV_RAND_NORMAL, cvScalar(0), cvScalar(1e-3));
       cvRandArr(&rng, &_dT1, CV_RAND_NORMAL, cvScalar(0), cvScalar(1e-4));
       cvRandArr(&rng, &_dT2, CV_RAND_NORMAL, cvScalar(0), cvScalar(1e-4));

       cvComposeRT( &_om1, &_T1, &_om2, &_T2, &_om3, &_T3,
           &_dom3dom1, &_dom3dT1, &_dom3dom2, &_dom3dT2,
           &_dT3dom1, &_dT3dT1, &_dT3dom2, &_dT3dT2 );

       cvMatMulAdd(&_dom3dom1, &_dom1, &_om3, &_om3p);
       cvMatMulAdd(&_dom3dT1, &_dT1, &_om3p, &_om3p);
       cvMatMulAdd(&_dom3dom2, &_dom2, &_om3p, &_om3p);
       cvMatMulAdd(&_dom3dT2, &_dT2, &_om3p, &_om3p);

       cvMatMulAdd(&_dT3dom1, &_dom1, &_T3, &_T3p);
       cvMatMulAdd(&_dT3dT1, &_dT1, &_T3p, &_T3p);
       cvMatMulAdd(&_dT3dom2, &_dom2, &_T3p, &_T3p);
       cvMatMulAdd(&_dT3dT2, &_dT2, &_T3p, &_T3p);

       cvAdd(&_om1, &_dom1, &_om1);
       cvAdd(&_om2, &_dom2, &_om2);
       cvAdd(&_T1, &_dT1, &_T1);
       cvAdd(&_T2, &_dT2, &_T2);

       cvComposeRT( &_om1, &_T1, &_om2, &_T2, &_om3r, &_T3r);

       double omerr = cvNorm(&_om3r, &_om3p, CV_L2)/(cvNorm(&_om3r,&_om3,CV_L2)+DBL_EPSILON);
       double Terr = cvNorm(&_T3r, &_T3p, CV_L2)/(cvNorm(&_T3r,&_T3,CV_L2)+DBL_EPSILON);
       printf("%d. om err = %g, T err = %g\n", iter, omerr, Terr);
       if( omerr > eps || Terr > eps )
           break;
   }
   printf( iter >= maxIter ? "\nOK\n" : "\nFAIL\n" );
}


static void
testStereoCalib(const char* imageList, int nx, int ny)
{
   int displayCorners = 0;
   int showUndistorted = 1;
   const int maxScale = 5;
   const float squareSize = 1.f;
   FILE* f = fopen(imageList, "rt");
   int i, j, lr, nframes, n = nx*ny;
   vector<string> imageNames[2];
   vector<CvPoint3D32f> objectPoints;
   vector<CvPoint2D32f> points[2];
   vector<int> npoints;
   vector<uchar> active[2];
   vector<CvPoint2D32f> temp(n);
   CvSize imageSize = {0,0};
   double K1[3][3], K2[3][3], D1[4], D2[4], R[3][3], T[3], E[3][3], F[3][3];
   CvMat _K1 = cvMat(3, 3, CV_64F, K1 );
   CvMat _K2 = cvMat(3, 3, CV_64F, K2 );
   CvMat _D1 = cvMat(1, 4, CV_64F, D1 );
   CvMat _D2 = cvMat(1, 4, CV_64F, D2 );
   CvMat _R = cvMat(3, 3, CV_64F, R );
   CvMat _T = cvMat(3, 1, CV_64F, T );
   CvMat _E = cvMat(3, 3, CV_64F, E );
   CvMat _F = cvMat(3, 3, CV_64F, F );

   if( displayCorners )
       cvNamedWindow( "corners", 1 );
   if( !f )
   {
       fprintf(stderr, "can not open file %s\n", imageList );
       return;
   }

   for(i=0;;i++)
   {
       char buf[1024];
       int count = 0, result=0, N;
       lr = i % 2;
       vector<CvPoint2D32f>& pts = points[lr];
       if( !fgets( buf, sizeof(buf)-3, f ))
           break;
       size_t len = strlen(buf);
       while( len > 0 && isspace(buf[len-1]))
           buf[--len] = '\0';
       if( buf[0] == '#' )
           continue;
       IplImage* img = cvLoadImage( buf, 0 );
       if( !img )
           break;
       imageSize = cvGetSize(img);
       imageNames[lr].push_back(buf);

       for( int s = 1; s <= maxScale; s++ )
       {
           IplImage* timg = img;
           if( s > 1 )
           {
               timg = cvCreateImage( cvSize(img->width*s, img->height*s), img->depth, img->nChannels );
               cvResize( img, timg, CV_INTER_CUBIC );
           }
           result = cvFindChessboardCorners( timg, cvSize(nx, ny), &temp[0], &count );
           if( timg != img )
               cvReleaseImage( &timg );
           if( result || s == maxScale )
               for( j = 0; j < count; j++ )
               {
                   temp[j].x /= s;
                   temp[j].y /= s;
               }
           if( result )
               break;
       }

       if( displayCorners )
       {
           printf("%s\n", buf);
           IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
           cvCvtColor( img, cimg, CV_GRAY2BGR );
           cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0], count, result );
           cvShowImage( "corners", cimg );
           cvReleaseImage( &cimg );
           cvWaitKey(0);
       }
       else
           putchar('.');

       N = pts.size();
       pts.resize(N + n, cvPoint2D32f(0,0));
       active[lr].push_back((uchar)result);
       //assert( result != 0 );
       if( result )
       {
           cvFindCornerSubPix( img, &temp[0], count, cvSize(11, 11), cvSize(-1,-1),
               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );
           copy( temp.begin(), temp.end(), pts.begin() + N );
       }
       cvReleaseImage( &img );
   }
   fclose(f);
   printf("\n");

   nframes = active[0].size();

   objectPoints.resize(nframes*n);
   for( i = 0; i < ny; i++ )
       for( j = 0; j < nx; j++ )
           objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
   for( i = 1; i < nframes; i++ )
       copy( objectPoints.begin(), objectPoints.begin() + n, objectPoints.begin() + i*n );

   npoints.resize(nframes,n);
   active[0].resize(nframes, (uchar)1);
   active[1].resize(nframes, (uchar)1);

   CvMat _objectPoints = cvMat(1, objectPoints.size(), CV_32FC3, &objectPoints[0] );
   CvMat _imagePoints1 = cvMat(1, points[0].size(), CV_32FC2, &points[0][0] );
   CvMat _imagePoints2 = cvMat(1, points[1].size(), CV_32FC2, &points[1][0] );
   CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
   CvMat _activeImages1 = cvMat( 1, active[0].size(), CV_8U, &active[0][0] );
   CvMat _activeImages2 = cvMat( 1, active[1].size(), CV_8U, &active[1][0] );

   cvSetIdentity(&_K1);
   cvSetIdentity(&_K2);

   cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,
       &_activeImages1, &_activeImages2, 0, 0, &_K1, &_D1, &_K2, &_D2,
       imageSize, &_R, &_T, &_E, &_F, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1, 1e-5),
       50, 50, CV_CALIB_FIX_PRINCIPAL_POINT+CV_CALIB_ZERO_TANGENT_DIST+CV_CALIB_FIX_ASPECT_RATIO);

   // optionally, average the output intrinsic parameters (assuming that both cameras are similar)
   // and repeat the [R|t] estimation procedure
   cvAddWeighted( &_K1, 0.5, &_K2, 0.5, 0, &_K1 );
   cvCopy( &_K1, &_K2 );
   cvAddWeighted( &_D1, 0.5, &_D2, 0.5, 0, &_D1 );
   cvCopy( &_D1, &_D2 );

   cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,
       &_activeImages1, &_activeImages2, 0, 0, &_K1, &_D1, &_K2, &_D2,
       imageSize, &_R, &_T, &_E, &_F, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 200, 1e-5),
       50, 50,
       //CV_CALIB_FIX_PRINCIPAL_POINT+CV_CALIB_ZERO_TANGENT_DIST+CV_CALIB_FIX_ASPECT_RATIO);
       CV_CALIB_FIX_INTRINSIC);

   // because the output fundamental matrix implicitly includes all the output information,
   // we can sort of check the correctness using the epipolar geometry constraint: m2^t*F*m1=0
   vector<CvPoint3D32f> lines[2];

   int nactive = 0;
   for( i = 0; i < nframes; i++ )
   {
       assert( (active[0][i] != 0) == (active[1][i] != 0) );
       nactive += active[0][i] != 0;
   }

   // compress image points
   for( i = j = 0; i < nframes; i++ )
   {
       if( active[0][i] != 0 )
       {
           if( i > j )
               for( lr = 0; lr < 2; lr++ )
                   copy( points[lr].begin() + i*n, points[lr].begin() + (i+1)*n, points[lr].begin() + j*n );
           j++;
       }
   }
   points[0].resize(nactive*n);
   points[1].resize(nactive*n);
   _imagePoints1 = cvMat(1, points[0].size(), CV_32FC2, &points[0][0] );
   _imagePoints2 = cvMat(1, points[1].size(), CV_32FC2, &points[1][0] );

   lines[0].resize(nactive*n);
   lines[1].resize(nactive*n);
   CvMat _L1 = cvMat(1, lines[0].size(), CV_32FC3, &lines[0][0]);
   CvMat _L2 = cvMat(1, lines[1].size(), CV_32FC3, &lines[1][0]);

   cvUndistortPoints( imageSize, &_imagePoints1, &_imagePoints1, &_K1, &_D1 );
   cvUndistortPoints( imageSize, &_imagePoints2, &_imagePoints2, &_K2, &_D2 );

   cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
   cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );

   double avgErr = 0;
   for( i = 0; i < nactive*n; i++ )
   {
       double err = fabs(points[0][i].x*lines[1][i].x + points[0][i].y*lines[1][i].y + lines[1][i].z) +
           fabs(points[1][i].x*lines[0][i].x + points[1][i].y*lines[0][i].y + lines[0][i].z);
       avgErr += err;
   }

   printf( "avg err = %g\n", avgErr/(nframes*n) );

   if( showUndistorted )
   {
       double H1[3][3], H2[3][3];
       CvMat _H1 = cvMat(3, 3, CV_64F, H1), _H2 = cvMat(3, 3, CV_64F, H2);
       CvMat* mx1 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
       CvMat* my1 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
       CvMat* mx2 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
       CvMat* my2 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
       CvMat* pair = cvCreateMat( imageSize.height, imageSize.width*2, CV_8UC3 );
       cvStereoCalcHomographiesFromF( imageSize, &_imagePoints1, &_imagePoints2, &_F, &_H1, &_H2, 3 );
       cvStereoInitUndistortRectifyMap( &_K1, &_D1, &_H1, mx1, my1 );
       cvStereoInitUndistortRectifyMap( &_K2, &_D2, &_H2, mx2, my2 );

       cvNamedWindow( "rectified", 1 );

       for( i = 0; i < nframes; i++ )
       {
           IplImage* img1 = cvLoadImage( imageNames[0][i].c_str(), 1 );
           IplImage* img2 = cvLoadImage( imageNames[1][i].c_str(), 1 );
           if( img1 && img2 )
           {
               CvMat part;
               cvGetCols( pair, &part, 0, imageSize.width );
               cvRemap( img1, &part, mx1, my1 );
               cvGetCols( pair, &part, imageSize.width, imageSize.width*2 );
               cvRemap( img2, &part, mx2, my2 );
               for( j = 0; j < imageSize.height; j += 16 )
                   cvLine( pair, cvPoint(0,j), cvPoint(imageSize.width*2,j), CV_RGB(0,255,0));
               cvShowImage( "rectified", pair );
               if( cvWaitKey() == 27 )
                   break;
           }
           cvReleaseImage( &img1 );
           cvReleaseImage( &img2 );
       }
       cvReleaseMat( &mx1 );
       cvReleaseMat( &my1 );
       cvReleaseMat( &mx2 );
       cvReleaseMat( &my2 );
   }
}

#if 0
void main(void)
{
   testStereoCalib("list.txt", 9, 6);
}
#endif