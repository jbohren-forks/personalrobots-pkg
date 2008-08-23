#include <iostream.h>
#include "CvMatUtils.h"
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#define DEBUG

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
