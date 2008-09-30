#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <string>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include "PoseEstimateDisp.h"
#include <star_detector/include/keypoint.h>
#include <cmath>
#include <cvwimage.h>
using namespace cv;

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
	static CvPoint3D64f rowToPoint(const CvMat& mat, int row);
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

	/// draw two collection of points onto canvas. first one in red circle, second one yellow cross.
	static bool drawPoints(cv::WImage3_b& image, vector<Keypoint>& keypoints0,
	    vector<Keypoint>& keypoints1);
	static bool drawMatchingPairs(CvMat& pts0, CvMat& pts1, cv::WImage3_b& canvas,
	    const CvMat& rot, const CvMat& shift,
	    const cv::willow::PoseEstimateDisp& peDisp, bool reversed=true);
  /// Draw a line between two points (in disparity space) of each pair onto canvas.
  /// The line is in red.
  static bool drawLines(
      /// pairs of points in disparity space
      WImage3_b & canvas, const vector<pair<CvPoint3D64f,CvPoint3D64f> > & pointPairsInDisp);
  /// Draw a line between two points (in disparity space) of each pair onto canvas.
  /// The line is in red.
  static bool drawLines(
      WImage3_b & canvas,
      /// index pairs of points, into keypoints0 and keypoints1, respectively
      const vector<pair<int,int> > & pointPairIndices,
      /// key point list 0 in disparity space
      const vector<Keypoint>& keypoints0,
      /// key point list 1 in disparity space
      const vector<Keypoint>& keypoints1
  );
  /// Convert disparity coordinate into pixel location in left cam CvPoint
  static inline CvPoint disparityToLeftCam(
      /// coordinate in disparity coordinates
      const CvPoint3D64f& dispCoord) {
    return cvPoint(
        (int)std::floor(dispCoord.x + .5),
        (int)std::floor(dispCoord.y + .5)
    );
  }
  /// Convert disparity coordinate into pixel location in left cam CvPoint
  static inline CvPoint disparityToRightCam(
      /// coordinate in disparity coordinates
      const CvPoint3D64f& dispCoord) {
    return cvPoint(
        (int)std::floor(dispCoord.x + .5),
        (int)std::floor(dispCoord.y - dispCoord.z + .5)
    );
  }

  /// Construct a transformation matrix (4x4 or 4x3), given
  /// the rodrigues and translation vector
  static void TransformationFromRodriguesAndShift(
      /// 6x1 matrix. The first 3 rows are the Rodrigues, the last 3 translation
      /// vector.
      const CvMat& param,
      /// Output. transformation matrix.
      CvMat& Transform);

	static const CvScalar red;
	static const CvScalar green;
	static const CvScalar yellow;
};

#endif /*CVMATUTILS_H_*/
