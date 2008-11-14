#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <string>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include "PoseEstimateDisp.h"
#include "VisOdom.h"

#include <cmath>

namespace cv {namespace willow {

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
	static bool drawPoints(cv::WImage3_b& image, const Keypoints& keypoints0,
	    const Keypoints& keypoints1);
	static bool drawMatchingPairs(CvMat& pts0, CvMat& pts1, cv::WImage3_b& canvas,
	    const CvMat& rot, const CvMat& shift,
	    const CvStereoCamModel& stCamModel, bool reversed);
  /// Draw a line between two points (in disparity space) of each pair onto canvas.
  /// The line is in red.
  static bool drawLines(
      /// pairs of points in disparity space
      WImage3_b & canvas, const vector<pair<CvPoint3D64f,CvPoint3D64f> > & pointPairsInDisp,
      const CvScalar color = CvMatUtils::red);
  /// Draw a line between two points (in disparity space) of each pair onto canvas.
  /// The line is in red.
  static bool drawLines(
      WImage3_b & canvas,
      /// index pairs of points, into keypoints0 and keypoints1, respectively
      const vector<pair<int,int> > & pointPairIndices,
      /// key point list 0 in disparity space
      const Keypoints& keypoints0,
      /// key point list 1 in disparity space
      const Keypoints& keypoints1,
      const CvScalar color = CvMatUtils::red
  );
  /// Given a disparity map and  location, returns the disparity value.
  /// The caller needs to make sure the location coordinates are valid.
  static inline double getDisparity(WImage1_16s& dispMap, CvPoint& pt,
      double disparityUnitInPixels=DefDisparityUnitInPixels) {
    // the unit of disp is 1/16 of a pixel - mDisparityUnitInPixels
    return CV_IMAGE_ELEM(dispMap.Ipl(), int16_t, pt.y, pt.x)/disparityUnitInPixels;
  }

  static CvMat* dispMapToMask(const WImage1_16s& dispMap);

  /// invert a rigid transformation matrix, 4x3 or 4x4
  static void invertRigidTransform(const CvMat* transf, CvMat* inv_transf);

  /**
   *  A convenient utility to construct a 4x4 transformation matrix
   *  from a 3x3 rotation matrix and a 3x1 translation matrix
   */
  static void transformFromRotationAndShift(
      /// 3x3 rotation matrix
      const CvMat& rot,
      /// 3x1 translation matrix
      const CvMat& shift,
      /// 4x4 transformation matrix
      CvMat& transform);

  /// Construct a transformation matrix (4x4 or 4x3), given
  /// the rodrigues and translation vector
  static void transformFromRodriguesAndShift(
      /// 6x1 matrix. The first 3 rows are the Rodrigues, the last 3 translation
      /// vector.
      const CvMat& param,
      /// Output. transformation matrix.
      CvMat& Transform);

  /// Construct rodrigues and shift vectors from 4x4
  /// transformation matrix
  static void transformToRodriguesAndShift(
      const CvMat& transform,
      /// 6x1 matrix. The first 3 rows are the Rodrigues, the last 3 translation
      /// vector.
      CvMat& params);

  static void loadStereoImagePair(string& dirname, string& leftimagefmt,
      string& rightimagefmt, string& dispmapfmt, int & frameIndex,
      WImageBuffer1_b* leftImage, WImageBuffer1_b* rightImage,
      WImageBuffer1_16s* dispMap);

  static const CvScalar red;
	static const CvScalar green;
	static const CvScalar yellow;
	static const CvScalar blue;
	static const CvScalar magenta;
};

}
}
#endif /*CVMATUTILS_H_*/
