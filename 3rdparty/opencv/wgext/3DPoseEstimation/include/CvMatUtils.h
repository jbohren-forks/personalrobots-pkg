#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <string>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include "Cv3DPoseEstimateDisp.h"

class CvMatUtils
{
public:
	CvMatUtils();
	virtual ~CvMatUtils();
    static void printMat(const CvMat *mat, const char *format="%12.5f,");

    static bool eulerAngle(const CvMat& rot, CvPoint3D64f &euler);
    static bool getVisualizableDisparityMap(
    		cv::WImage1_16s& dispMap, cv::WImage3_b& canvas, double maxDisp);
    static bool showDisparityMap(cv::WImage1_16s& dispMap, std::string& windowName, std::string& filename,
    		double maxDisp);

    static void cvCross(CvArr* img, CvPoint pt, int halfLen, CvScalar color,
            int thickness=1, int line_type=8, int shift=0);

    static bool drawMatchingPairs(CvMat& pts0, CvMat& pts1, cv::WImage3_b& canvas,
    		CvMat& rot, CvMat& shift, Cv3DPoseEstimateDisp& peDisp, bool reversed=true);

	static const CvScalar red;
	static const CvScalar green;
	static const CvScalar yellow;
};

#if 0
/**
 *   Trying to copy WImage
 */
namespace cv {
template<typename T> WGMat;
template <typename T> WGMatBuffer;

template<typename T>
class WGMatBuffer : public WGMat<T>
{
public:
	typename WGMat<T>::BaseType BaseType;

};
}
#endif
#endif /*CVMATUTILS_H_*/
