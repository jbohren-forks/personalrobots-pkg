#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <string>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>

typedef double CvMyReal;
#define CV_XF CV_64F

class CvMatUtils
{
public:
	CvMatUtils();
	virtual ~CvMatUtils();
    static void printMat(const CvMat *mat, const char *format="%12.5f,");
    static bool getVisualizableDisparityMap(
    		cv::WImage1_16s& dispMap, cv::WImage3_b& canvas, double maxDisp);
    static bool showDisparityMap(cv::WImage1_16s& dispMap, std::string& windowName, std::string& filename,
    		double maxDisp);
};

#endif /*CVMATUTILS_H_*/
