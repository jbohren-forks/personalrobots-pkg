
#include "CvMatUtils.h"
using namespace cv::willow;

#include <iostream>
#include <fstream>
using namespace std;

#include <limits.h>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "CvMat3X3.h"

#include <boost/foreach.hpp>
#include <Eigen/Geometry>
#undef DEBUG

const CvScalar CvMatUtils::red    = CV_RGB(255, 0, 0);
const CvScalar CvMatUtils::green  = CV_RGB(0, 255, 0);
const CvScalar CvMatUtils::yellow = CV_RGB(255, 255, 0);
const CvScalar CvMatUtils::blue   = CV_RGB(0, 0, 255);
const CvScalar CvMatUtils::magenta= CV_RGB(255, 0, 255);

CvMatUtils::CvMatUtils()
{
}

CvMatUtils::~CvMatUtils()
{
}

void CvMatUtils::printMat(const CvMat *mat, const char * format){
  if (format == NULL) format = "%12.5f";
  cout << "A Matrix of "<<mat->rows<<" by "<< mat->cols <<endl;
  for (int i=0; i<mat->rows; i++) {
    for (int j=0; j<mat->cols; j++) {
      printf(format, cvmGet(mat, i, j));
      printf(" ");
    }
    cout << endl;
  }
}

void CvMatUtils::saveMat(const char* filename, const CvMat *mat, const char* msg, const char * format) {
  if (format == NULL) format = "%12.5f";
  FILE* fp = fopen(filename, "w");

  fprintf(fp, "# %s\n", msg);

  for (int i=0; i<mat->rows; i++) {
    for (int j=0; j<mat->cols; j++) {
      fprintf(fp,format, cvmGet(mat, i, j));
      fprintf(fp," ");
    }
    fprintf(fp,"\n");
  }
  fclose(fp);
}

// the unit of the dispMap is 1/16 of a pixel
bool CvMatUtils::getVisualizableDisparityMap(cv::WImage1_16s& dispMap, cv::WImage3_b& canvas,
		double maxDisp) {
	bool status = true;

	double minVal, maxVal;
	int xim = dispMap.Width();
	int yim = dispMap.Height();
	cvMinMaxLoc(dispMap.Ipl(), &minVal, &maxVal);
#ifdef DEBUG
	printf("min, max of dispImg: %f, %f\n", minVal/16., maxVal/16.);
#endif
	uint8_t *_dispImgU8C3 = canvas.ImageData();
	const double gamma = 0.4;  // between 0.0 and 1.0
	for (int v=0; v<yim; v++) {
		for (int u=0; u<xim; u++) {
			CvScalar s = cvGet2D(dispMap.Ipl(), v, u);
			double d = s.val[0]/16.0;
			if (d < 0) {
				// set it to blue (BGR)
				_dispImgU8C3[(v*xim+u)*3 + 0] = 255;
				_dispImgU8C3[(v*xim+u)*3 + 1] = 0;
				_dispImgU8C3[(v*xim+u)*3 + 2] = 0;
			} else if (d==0) {
				// set it to yellow (BGR)
				_dispImgU8C3[(v*xim+u)*3 + 0] = 0;
				_dispImgU8C3[(v*xim+u)*3 + 1] = 255;
				_dispImgU8C3[(v*xim+u)*3 + 2] = 255;
			} else if (d > maxDisp) {
				// set it to red (BGR)
				_dispImgU8C3[(v*xim+u)*3 + 0] = 0;
				_dispImgU8C3[(v*xim+u)*3 + 1] = 0;
				_dispImgU8C3[(v*xim+u)*3 + 2] = 255;
			} else {
				uint8_t gray = d/(double)maxDisp * 255.;
				gray = (int)(0.5 + 255.0 * pow((double)gray/255.0, gamma));
				_dispImgU8C3[(v*xim+u)*3 + 0] = gray;
				_dispImgU8C3[(v*xim+u)*3 + 1] = gray;
				_dispImgU8C3[(v*xim+u)*3 + 2] = gray;
			}
		}
	}

	return status;
}

bool CvMatUtils::showDisparityMap(cv::WImage1_16s& dispMap, std::string& windowName, std::string& filename,
		double maxDisp) {
	bool status = true;

	cv::WImageBuffer3_b canvas(dispMap.Width(), dispMap.Height());

	CvMatUtils::getVisualizableDisparityMap(dispMap, canvas, maxDisp);
	cvShowImage(windowName.c_str(),  canvas.Ipl());
	cvSaveImage(filename.c_str(),    canvas.Ipl());
	return status;
}

void CvMatUtils::cvCross(CvArr* img, CvPoint pt, int halfLen, CvScalar color,
        int thickness, int line_type, int shift) {
	CvPoint pt1;
	CvPoint pt2;
	pt1.x = pt.x - halfLen;
	pt2.x = pt.x + halfLen;
	pt1.y = pt2.y = pt.y;
	cvLine(img, pt1, pt2, color, thickness, line_type, shift);
	pt1.x = pt2.x = pt.x;
	pt1.y = pt.y - halfLen;
	pt2.y = pt.y + halfLen;
	cvLine(img, pt1, pt2, color, thickness, line_type, shift);
}

bool CvMatUtils::drawPoints(cv::WImage3_b& image, const Keypoints& keyPointsLast,
    const Keypoints& keyPointsCurr){
  // draw the key points
  IplImage* img = image.Ipl();
  BOOST_FOREACH(const CvPoint3D64f& p, keyPointsCurr) {
    cvCircle(img, cvPoint(p.x, p.y), 4, CvMatUtils::green, 1, CV_AA, 0);
  }
  // draw the key points from last key frame
  BOOST_FOREACH( const CvPoint3D64f& p, keyPointsLast ){
    // draw cross instead of circle
    CvMatUtils::cvCross(img, cvPoint(p.x, p.y), 4, CvMatUtils::yellow, 1, CV_AA, 0);
  }
  return true;
}

bool CvMatUtils::drawMatchingPairs(CvMat& pts0, CvMat& pts1, cv::WImage3_b& canvas,
		const CvMat& rot, const CvMat& shift,
		const CvStereoCamModel& stCamModel, bool reversed) {
	int numInliers = pts0.rows;
	if (pts1.rows != numInliers) {
		cerr << __PRETTY_FUNCTION__ << "matching pairs do not match in length"<<endl;
		return false;
	}

	double _xyzs0[3*numInliers];
	double _xyzs0To1[3*numInliers];
	double _uvds0To1[3*numInliers];
	double _xyzs1[3*numInliers];
	CvMat xyzs0    = cvMat(numInliers, 3, CV_64FC1, _xyzs0);
	CvMat xyzs0To1 = cvMat(numInliers, 3, CV_64FC1, _xyzs0To1);
	CvMat uvds0To1 = cvMat(numInliers, 3, CV_64FC1, _uvds0To1);
	CvMat xyzs1    = cvMat(numInliers, 3, CV_64FC1, _xyzs1);

	stCamModel.dispToCart(&pts0, &xyzs0);
	stCamModel.dispToCart(&pts1, &xyzs1);

	if (reversed == true) {
	  // compute the inverse transformation
	  double _invRot[9], _invShift[3];
	  CvMat invRot   = cvMat(3, 3, CV_64FC1, _invRot);
	  CvMat invShift = cvMat(3, 1, CV_64FC1, _invShift);

	  cvTranspose(&rot, &invRot);
	  cvGEMM(&invRot, &shift, -1., NULL, 0., &invShift, 0.0);
	  CvMat xyzs0Reshaped;
	  CvMat xyzs0To1Reshaped;
	  cvReshape(&xyzs0,    &xyzs0Reshaped, 3, 0);
	  cvReshape(&xyzs0To1, &xyzs0To1Reshaped, 3, 0);
	  cvTransform(&xyzs0Reshaped, &xyzs0To1Reshaped, &invRot, &invShift);
	} else {
	  CvMat xyzs0Reshaped;
	  CvMat xyzs0To1Reshaped;
	  cvReshape(&xyzs0,    &xyzs0Reshaped,    3, 0);
	  cvReshape(&xyzs0To1, &xyzs0To1Reshaped, 3, 0);
	  cvTransform(&xyzs0Reshaped, &xyzs0To1Reshaped, &rot, &shift);
	}

	stCamModel.cartToDisp(&xyzs0To1, &uvds0To1);
	IplImage* img = canvas.Ipl();

	// draw uvds0To1 on leftimgeC3a
	for (int k=0;k<numInliers;k++) {
		CvPoint pt0To1 = cvPoint((int)(_uvds0To1[k*3+0]+.5), (int)(_uvds0To1[k*3+1] + .5));
		const int halfLen = 4;
		cvCross(img, pt0To1, halfLen, CvMatUtils::yellow);
		CvPoint pt1 = cvPoint((int)(cvGetReal2D(&pts1, k, 0)+.5), (int)(cvGetReal2D(&pts1, k, 1)+.5));
		cvCircle(img, pt1, 4, CvMatUtils::green, 1, CV_AA, 0);
		cvLine(img, pt1, pt0To1, CvMatUtils::red, 1, CV_AA, 0);
	}
	return true;
}

/**
 * a convenient function to convert from rotation matrix to euler angles.
 */
bool CvMatUtils::rotMatToEuler(const CvMat& rot, CvPoint3D64f& euler) {
	double _R[9], _Q[9];
	CvMat R, Q;
	CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
	cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

	cvRQDecomp3x3(&rot, &R, &Q, pQx, pQy, pQz, &euler);
	// note that the euler angles are in degrees. convert them to radians
	euler.x *= CV_PI/180.;
	euler.y *= CV_PI/180.;
	euler.z *= CV_PI/180.;
	return true;
}

void CvMatUtils::transformFromEulerAndShift(const CvMat* params,
    CvMat* transform) {
  double p[6];
  for (int r=0; r<6; r++) {
    p[r] = cvmGet(params, r, 0);
  }

  CvMat rot;
  double _rot[9];
  CvMat rot0 = cvMat(3, 3, CV_64FC1, _rot);
  cvGetSubRect(transform, &rot, cvRect(0, 0, 3, 3));

  CvMat3X3<double>::rotMatrix(p[0], p[1], p[2], _rot,
      CvMat3X3<double>::EulerXYZ);
  cvCopy(&rot0, &rot);

  cvmSet(transform, 0, 3, p[3]);
  cvmSet(transform, 1, 3, p[4]);
  cvmSet(transform, 2, 3, p[5]);
}

void CvMatUtils::invertRigidTransform(const CvMat* transf, CvMat* inv_transf) {
  CvMat rot, inv_rot;
  CvMat shift, inv_shift;

  cvGetSubRect(transf, &rot,   cvRect(0, 0, 3, 3));
  cvGetSubRect(transf, &shift, cvRect(3, 0, 1, 3));
  cvGetSubRect(inv_transf, &inv_rot,   cvRect(0, 0, 3, 3));
  cvGetSubRect(inv_transf, &inv_shift, cvRect(3, 0, 1, 3));
  // rotation matrix is the transpose of the input one.
  cvTranspose(&rot, &inv_rot);
  // translation vector is the new rotation matrix times the negative of
  // the input translation vector.
  cvGEMM(&inv_rot, &shift, -1.0, NULL, 0.0, &inv_shift, 0);

  if (inv_transf->rows==4) {
    cvmSet(inv_transf, 3, 0, 0.);
    cvmSet(inv_transf, 3, 1, 0.);
    cvmSet(inv_transf, 3, 2, 0.);
    cvmSet(inv_transf, 3, 3, 1.);
  }
}

void CvMatUtils::transformFromRodriguesAndShift(const CvMat* param, CvMat* transform) {
  CvMat rod;
  CvMat rot;
  CvMat shift;
  CvMat shiftInParam;
  assert(param->rows==6 && param->cols==1);
  cvGetRows(param, &rod, 0, 3);
  cvGetRows(param, &shiftInParam, 3, 6);
  // get a view to the 3x3 sub matrix for rotation
  cvGetSubRect(transform,  &rot, cvRect(0,0, 3, 3));

  cvRodrigues2(&rod, &rot);

  // get a view to the 3x1 submatrix for translation
  cvGetSubRect(transform, &shift, cvRect(3, 0, 1, 3));
  cvCopy(&shiftInParam, &shift);

  if (transform->rows==4) {
    cvmSet(transform, 3, 0, 0.);
    cvmSet(transform, 3, 1, 0.);
    cvmSet(transform, 3, 2, 0.);
    cvmSet(transform, 3, 3, 1.);
  }
}

void CvMatUtils::transformToRodriguesAndShift(
    const CvMat& transform,
    /// 6x1 matrix. The first 3 rows are the Rodrigues, the last 3 translation
    /// vector.
    CvMat& params) {
  CvMat rot;
  CvMat shift;
  CvMat rod;
  CvMat shift1;
  cvGetRows(&params, &rod,    0, 3);
  cvGetRows(&params, &shift1, 3, 6);
  cvGetSubRect(&transform, &rot, cvRect(0, 0, 3, 3));
  cvRodrigues2(&rot, &rod);

  cvGetSubRect(&transform, &shift, cvRect(3, 0, 1, 3));
  cvCopy(&shift, &shift1);
}

void CvMatUtils::transformToQuaternionAndShift(
    /// 4x4 matrix or 3x4 matrix of transformation. CV_64FC1 or CV_32FC1
    const CvMat* transform,
    /// 7x1 matrix. The first 4 rows are the quaternion, the last 3 translation
    /// vector.
    CvMat* params){
  Eigen::Matrix3d rot;

  for (int i=0;i<3; i++) {
    for (int j=0; j<3; j++) {
      rot(i,j) = cvmGet(transform, i, j);
    }
  }
  Eigen::Quaterniond quatd(rot);

  // quaternion
  cvmSet(params, 0, 0, quatd.w());
  cvmSet(params, 1, 0, quatd.x());
  cvmSet(params, 2, 0, quatd.y());
  cvmSet(params, 3, 0, quatd.z());

  // translation
  cvmSet(params, 4, 0, cvmGet(transform, 0, 3));
  cvmSet(params, 5, 0, cvmGet(transform, 1, 3));
  cvmSet(params, 6, 0, cvmGet(transform, 2, 3));
}

void CvMatUtils::transformFromQuaternionAndShift(
    /// 7x1 matrix. The first 4 rows are the quaternion, the last 3 translation
    const CvMat* params,
    CvMat *transform
    ){
  double w = cvmGet(params, 0, 0);
  double x = cvmGet(params, 1, 0);
  double y = cvmGet(params, 2, 0);
  double z = cvmGet(params, 3, 0);
  Eigen::Quaterniond quatd(w, x, y, z);
  Eigen::Matrix3d rot = quatd.toRotationMatrix();


  cvmSet(transform, 0, 0, rot(0,0));
  cvmSet(transform, 0, 1, rot(0,1));
  cvmSet(transform, 0, 2, rot(0,2));
  cvmSet(transform, 1, 0, rot(1,0));
  cvmSet(transform, 1, 1, rot(1,1));
  cvmSet(transform, 1, 2, rot(1,2));
  cvmSet(transform, 2, 0, rot(2,0));
  cvmSet(transform, 2, 1, rot(2,1));
  cvmSet(transform, 2, 2, rot(2,2));

  // fill in the translation part
  CvMat shift0, shift1;
  cvGetRows(params, &shift0, 4, 7);
  cvGetSubRect(transform, &shift1, cvRect(3, 0, 1, 3));
  cvCopy(&shift0, &shift1);

  if (transform->rows == 4) {
    cvmSet(transform, 3, 0, 0.);
    cvmSet(transform, 3, 1, 0.);
    cvmSet(transform, 3, 2, 0.);
    cvmSet(transform, 3, 3, 1.);
  }
}


CvPoint3D64f CvMatUtils::rowToPoint(const CvMat& mat, int row){
  CvPoint3D64f coord;
  coord.x = cvmGet(&mat, row, 0);
  coord.y = cvmGet(&mat, row, 1);
  coord.z = cvmGet(&mat, row, 2);
  return coord;
}

bool CvMatUtils::drawLines(
    WImage3_b& canvas,
    const vector<pair<CvPoint3D64f, CvPoint3D64f> >& pointPairsInDisp,
    const CvScalar color){
  for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = pointPairsInDisp.begin();
  iter != pointPairsInDisp.end(); iter++) {
    const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
    CvPoint p0 = CvStereoCamModel::dispToLeftCam(p.first);
    CvPoint p1 = CvStereoCamModel::dispToLeftCam(p.second);
    int thickness =1;
    cvLine(canvas.Ipl(), p0, p1, color, thickness, CV_AA);
  }
  return true;
}
bool CvMatUtils::drawLines(
    WImage3_b& canvas,
    const vector<pair<int, int> >& indexPairs,
    const Keypoints& keypoints0,
    const Keypoints& keypoints1,
    const CvScalar color){
  for (vector<pair<int, int> >::const_iterator iter = indexPairs.begin();
  iter != indexPairs.end();
  iter++) {
    const pair<int, int>& p = *iter;
    CvPoint p0 = cvPoint(keypoints0[p.first].x, keypoints0[p.first].y);
    CvPoint p1 = cvPoint(keypoints1[p.second].x, keypoints1[p.second].y);
    int thickness =1;
    cvLine(canvas.Ipl(), p0, p1, color, thickness, CV_AA);
  }
  return true;
}

CvMat* CvMatUtils::dispMapToMask(const WImage1_16s& dispMap){
  CvSize sz = cvSize(dispMap.Width(), dispMap.Height());
  const int16_t *disp = dispMap.ImageData();
  int8_t* _mask = new int8_t[sz.width*sz.height];
  for (int v=0; v<sz.height; v++) {
    for (int u=0; u<sz.width; u++) {
      int16_t d = disp[v*sz.width+u];
      if (d>0) {
        _mask[v*sz.width+u] = 1;
      } else {
        _mask[v*sz.width+u] = 0;
      }
    }
  }
  CvMat* mat = new CvMat;
  mat = cvInitMatHeader( mat, sz.height, sz.width, CV_8SC1, _mask);
  return mat;
}

void CvMatUtils::loadStereoImagePair(string& dirname, string& leftimagefmt,
    string& rightimagefmt, string& dispmapfmt, int & frameIndex,
    WImageBuffer1_b* leftImage, WImageBuffer1_b* rightImage,
    WImageBuffer1_16s* dispMap
){
  char leftfilename[PATH_MAX];
  char rightfilename[PATH_MAX];
  char dispmapfilename[PATH_MAX];

  if (leftImage) {
    sprintf(leftfilename, leftimagefmt.c_str(),   frameIndex);
    IplImage* leftimg  = cvLoadImage(leftfilename,  CV_LOAD_IMAGE_GRAYSCALE);
    leftImage->SetIpl(leftimg);
  }
  if (rightImage) {
    sprintf(rightfilename, rightimagefmt.c_str(), frameIndex);
    IplImage* rightimg = cvLoadImage(rightfilename, CV_LOAD_IMAGE_GRAYSCALE);
    rightImage->SetIpl(rightimg);
  }
  if (dispMap) {
    sprintf(dispmapfilename, dispmapfmt.c_str(),  frameIndex);
    IplImage* dispimg = cvLoadImage(dispmapfilename, CV_LOAD_IMAGE_GRAYSCALE);
    dispMap->SetIpl(dispimg);
  }
#if DEBUG==1
  cout << "loaded " << leftfilename << " and " << rightfilename << endl;
#endif
}

void CvMatUtils::transformFromRotationAndShift(
    const CvMat& rot, const CvMat& shift, CvMat& transform) {
  // construct RT
  for (int r=0; r<3; r++) {
    for (int c=0; c<3; c++) {
      cvSetReal2D(&transform, r, c, cvmGet(&rot, r, c));
    }
    cvSetReal2D(&transform, r, 3, cvmGet(&shift, r, 0));
  }

  // last row
  cvSetReal2D(&transform, 3, 0, 0.0);
  cvSetReal2D(&transform, 3, 1, 0.0);
  cvSetReal2D(&transform, 3, 2, 0.0);
  cvSetReal2D(&transform, 3, 3, 1.0);
}

#if 0

/**
 * \brief Solve a linear equation by Cholesky factorization.
 * Not ready yet.
 */
bool CvMatUtils::SolveByCholeskyFact(const double *A, const double* b,
    double* x, int n){
  int i, j, k;
  double* buf = (double*)cvStackAlloc((n*(n+1)/2+n*2)*sizeof(buf[0]));
  double* L0 = buf + n, *L = L0;
  double* y = L + (n+1)*n/2;
  double s;

  // step one: compute L: A = L*L'
  for( i = 0; i < n; ++i, L += i, A += n )
  {
    double si = 0;
    const double* tL = L0;
    for( j = 0; j < i; ++j, tL += j )
    {
      for( k = 0, s = 0; k < j; k++ )
        s += L[k]*tL[k];
      L[j] = (A[j] - s)*buf[j];
      si += L[j]*L[j];
    }
    si = A[i] - si;
    if( si < DBL_EPSILON )
      return false;
    L[i] = sqrt(si);
    buf[i] = 1./L[i];
  }

  for( j = 0; j < n; j++ )
    y[j] = 0;

  // step two: solve Ly = b for y, then L'x = y for x.

  return true;
}


#endif

