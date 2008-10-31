
#include "CvMatUtils.h"
using namespace cv::willow;

#include <iostream.h>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/foreach.hpp>
#undef DEBUG

const CvScalar CvMatUtils::red    = CV_RGB(255, 0, 0);
const CvScalar CvMatUtils::green  = CV_RGB(0, 255, 0);
const CvScalar CvMatUtils::yellow = CV_RGB(255, 255, 0);

CvMatUtils::CvMatUtils()
{
}

CvMatUtils::~CvMatUtils()
{
}

void CvMatUtils::printMat(const CvMat *mat, const char * format){
	cout << "A Matrix of "<<mat->rows<<" by "<< mat->cols <<endl;
	for (int i=0; i<mat->rows; i++) {
		for (int j=0; j<mat->cols; j++) {
			printf(format, cvmGet(mat, i, j));
		}
		cout << endl;
	}
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
		const PoseEstimateDisp& pedisp, bool reversed) {
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

	pedisp.dispToCart(pts0, xyzs0);
	pedisp.dispToCart(pts1, xyzs1);

	if (reversed == true) {
		// compute the inverse transformation
		double _invRot[9], _invShift[3];
		CvMat invRot   = cvMat(3, 3, CV_64FC1, _invRot);
		CvMat invShift = cvMat(3, 1, CV_64FC1, _invShift);

		cvInvert(&rot, &invRot);
		cvGEMM(&invRot, &shift, -1., NULL, 0., &invShift, 0.0);
		CvMat xyzs0Reshaped;
		CvMat xyzs0To1Reshaped;
		cvReshape(&xyzs0,    &xyzs0Reshaped, 3, 0);
		cvReshape(&xyzs0To1, &xyzs0To1Reshaped, 3, 0);
		cvTransform(&xyzs0Reshaped, &xyzs0To1Reshaped, &invRot, &invShift);
	} else {
		CvMat xyzs0Reshaped;
		CvMat xyzs0To1Reshaped;
		cvReshape(&xyzs0,    &xyzs0Reshaped, 3, 0);
		cvReshape(&xyzs0To1, &xyzs0To1Reshaped, 3, 0);
		cvTransform(&xyzs0Reshaped, &xyzs0To1Reshaped, &rot, &shift);
	}

	pedisp.cartToDisp(xyzs0To1, uvds0To1);
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
bool CvMatUtils::eulerAngle(const CvMat& rot, CvPoint3D64f& euler) {
	double _R[9], _Q[9];
	CvMat R, Q;
	CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
	cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

	cvRQDecomp3x3(&rot, &R, &Q, pQx, pQy, pQz, &euler);
	return true;
}

void CvMatUtils::TransformationFromRodriguesAndShift(const CvMat& param, CvMat& transform) {
  CvMat rod;
  CvMat rot;
  CvMat shift;
  CvMat shiftInParam;
  assert(param.rows==6 && param.cols==1);
  cvGetRows(&param, &rod, 0, 3);
  cvGetRows(&param, &shiftInParam, 3, 6);
  // get a view to the 3x3 sub matrix for rotation
  cvGetSubRect(&transform,  &rot, cvRect(0,0, 3, 3));
  // rodrigues to rotation matrix
  cvRodrigues2(&rod, &rot);
  // get a view to the 3x1 submatrix for translation
  cvGetSubRect(&transform, &shift, cvRect(3, 0, 1, 3));
  cvCopy(&shiftInParam, &shift);
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
    const vector<pair<CvPoint3D64f, CvPoint3D64f> >& pointPairsInDisp){
  for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = pointPairsInDisp.begin();
  iter != pointPairsInDisp.end(); iter++) {
    const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
    CvPoint p0 = CvStereoCamModel::dispToLeftCam(p.first);
    CvPoint p1 = CvStereoCamModel::dispToLeftCam(p.second);
    int thickness =1;
    cvLine(canvas.Ipl(), p0, p1, CvMatUtils::red, thickness, CV_AA);
  }
  return true;
}
bool CvMatUtils::drawLines(
    WImage3_b& canvas,
    const vector<pair<int, int> >& indexPairs,
    const Keypoints& keypoints0,
    const Keypoints& keypoints1){
  for (vector<pair<int, int> >::const_iterator iter = indexPairs.begin();
  iter != indexPairs.end();
  iter++) {
    const pair<int, int>& p = *iter;
    CvPoint p0 = cvPoint(keypoints0[p.first].x, keypoints0[p.first].y);
    CvPoint p1 = cvPoint(keypoints1[p.second].x, keypoints1[p.second].y);
    int thickness =1;
    cvLine(canvas.Ipl(), p0, p1, CvMatUtils::red, thickness, CV_AA);
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
#ifdef DEBUG
  cout << "loading " << leftfilename << " and " << rightfilename << endl;
#endif
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
}



