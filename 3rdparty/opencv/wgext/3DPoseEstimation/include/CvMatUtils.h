#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <string>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include "Cv3DPoseEstimateDisp.h"

/**
 * Misc utilities
 */
class CvMatUtils
{
public:
	CvMatUtils();
	virtual ~CvMatUtils();
	/**
	 *  print a matrix to standard output
	 */
	static void printMat(const CvMat *mat, const char *format="%12.5f,");
	/**
	 *  Convert a rotation matrix to euler angles
	 */
	static bool eulerAngle(const CvMat& rot, CvPoint3D64f &euler);
	/**
	 * convert a disparity map to an image suitable for display
	 */
	static bool getVisualizableDisparityMap(
	    cv::WImage1_16s& dispMap, cv::WImage3_b& canvas, double maxDisp);
	static bool showDisparityMap(cv::WImage1_16s& dispMap, std::string& windowName, std::string& filename,
	    double maxDisp);

	/**
	 * draw a cross over an image
	 * @param img
	 * @param pt     - Center of the cross
	 * @param halfLen - half length of the bounding box.
	 * @param color   - color of the cross
	 * @param thickness - thickness of the stroke
	 * @param line_type - Type of the line
	 * @param shift     - shift
	 */
	static void cvCross(CvArr* img, CvPoint pt, int halfLen, CvScalar color,
	    int thickness=1, int line_type=8, int shift=0);

	static bool drawMatchingPairs(CvMat& pts0, CvMat& pts1, cv::WImage3_b& canvas,
	    CvMat& rot, CvMat& shift, Cv3DPoseEstimateDisp& peDisp, bool reversed=true);

	static const CvScalar red;
	static const CvScalar green;
	static const CvScalar yellow;
};

#endif /*CVMATUTILS_H_*/
