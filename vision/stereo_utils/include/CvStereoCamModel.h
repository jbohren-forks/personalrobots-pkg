#ifndef WGSTEREOCAMMODEL_H_
#define WGSTEREOCAMMODEL_H_

#include <cmath>

#include <opencv/cxtypes.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

//* CvStereoCamModel
/**
 * This class transforms to and fro disparity space and Cartesian space, on
 * sparse data points as well as images. The class also includes routines that
 * converts between a range in x (or y) in Cartesion space and a range in u (or v)
 * in disparity space, given a certain disparity (or depth), and routines that
 * converts between disparity values and depths.
 * As implied, the points and image in disparity space are rectified.
 * This class also provides a method to compute mask according to depth/disparity
 * ranges.
 */
class CvStereoCamModel
{
public:
	/// Constructor.
  /**
   * \brief Constructor that takes camera parameters and parameterizes the object.
   * The object is ready to be used after constructed by this constructor.
   *  @param Fx  - focal length in x direction of the rectified image in pixels.
   *  @param Fy  - focal length in y direction of the rectified image in pixels.
   *  @param Tx  - Translation in x direction from the left camera to the right camera.
   *  @param Clx - x coordinate of the optical center of the left  camera
   *  @param Crx - x coordinate of the optical center of the right camera
   *  @param Cy  - y coordinate of the optical center of both left and right camera
   *  @param dispUnitScale - the unit of the value in a disparity map. e.g. 1/4 of a pixel.
   */
  CvStereoCamModel(
      double Fx,
      double Fy,
      double Tx,
      double Clx,
      double Crx,
      double Cy,
      double dispUnitScale = DefaultDispUnitScale);

  /**
   * \brief Copy constructor.
   */
  CvStereoCamModel(const CvStereoCamModel& camModel);

  /**
   * \brief Default constructor.
   * At least a call of any overloaded method of setCameraParam() is needed
   * to parameterize this object, or else a CV_ERROR happens on any attempt
   * to use this object, as the object is not parameterized yet.
   */
  CvStereoCamModel();

  virtual ~CvStereoCamModel();

  /**
   * \brief Parameterized the object.
   * Transformation matrices are built after
   * this called. An object of this class is not usable unless it is constructed
   * by constructor that takes camera parameters, or after the first call
   * of one of the overloaded method of setCameraParams.
   *  @param Fx  - focal length in x direction of the rectified image in pixels.
   *  Valid value shall be greater than zero.
   *  @param Fy  - focal length in y direction of the rectified image in pixels.
   *  Valid value shall be greater than zero.
   *  @param Tx  - Translation in x direction from the left camera to the right camera.
   *  Valid value shall be greater than zero.
   *  @param Clx - x coordinate of the optical center of the left  camera.
   *  Valid value shall be greater than zero.
   *  @param Crx - x coordinate of the optical center of the right camera
   *  Valid value shall be greater than zero.
   *  @param Cy  - y coordinate of the optical center of both left and right camera
   *  Valid value shall be greater than zero.
   *  @param dispUnitScale - the unit of the value in a disparity map. e.g. 1/4 of a pixel.
   *  Valid value shall be greater than zero.
   */
  void setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx,
      double Cy, double dispUnitScale = DefaultDispUnitScale);
  void setCameraParams(const CvStereoCamModel& params);

  /*!
   * \brief Convert 3D points from disparity coordinates to Cartesian coordinates.
   * The input are assumed to be "good points", i.e. with proper disparity
   * values that are greater than zero.
   */
	void dispToCart(
      /// (Input) 3D points in disparity coordinates.
	    /// The points are either stored in rows, as Nx3 1-channel matrix,
	    /// or Nx1 3-channel matrix.
	    const CvMat* uvds,
      /// (Output) 3D points in Cartesian coordinates.
	    /// The points are either stored in rows, as Nx3 1-channel matrix,
	    /// or Nx1 3-channel matrix.
	    CvMat* XYZs) const;

	/**
	 * \brief Convert disparity image map to image of 3D points.
	 */
	void dispToCart(
    /// disparity map. Id has to be 16SC1
    const IplImage *Id,
    /// Output 3D image. Ixyz has to be 32F
		IplImage *Ixyz) const;

  /// MUCH FASTER CONVERSION OF POINTS AND PUTTING THEM INTO X, Y, and Z images.
	///Id is single channel 8U disparity (it accepts an ROI and if so will use that on all the images
	///ZnearMM, ZfarMM -- Near and far thresholds for depth to convert
	///Ix, Iy, Iz are single channel 32F images which will contain the results  All images must be same size
	void disp8UToCart32F(const IplImage *Id, float ZnearMM, float ZfarMM, IplImage *Iz, IplImage *Ix, IplImage *Iy) const;


  /// \brief Convert 3D points from Cartesian coordinates to disparity coordinates.
	void cartToDisp(
      /// (Input) 3D points stored in rows, in Cartesian coordinates.
	    const CvMat* XYZs,
      /// (Output) 3D points stored in rows, in disparity coordinates.
	    CvMat* uvds) const;

	/// \brief Get references to the projection matrices.
	void getProjectionMatrices(CvMat* cartToDisp, CvMat* dispToCart) const {
	  *cartToDisp = mat_cart_to_disp_;
	  *dispToCart = mat_disp_to_cart_;
	}

	/// \brief Compute delta u, given Z and delta X in Cartesian space.
	/// @return delta u if Z is non-zero. DBL_MAX if Z is 0
	double getDeltaU(double deltaX, double Z) const;
	/// \brief Compute delta X, given disparity and delta u in disparity space.
	/// \return  0 if disparity is 0, namely d-(Clx-Crx) == 0
	double getDeltaX(double deltaU, double d) const;
	/// \brief compute delta v, given Z and delta Y in Cartesian space.
	/// @return delta v if Z is nonzero. DBL_MAX if Z is 0
	double getDeltaV(double deltaY, double Z) const;
	/// @brief compute delta Y, given disparity and delta v in disparity space.
	/// @return delta v, or 0 if d-(Clx-Crx) == 0
	double getDeltaY(double deltaV, double d) const;
	/// @brief compute Z given disparity.
	/// Symbolically, Fx_*Tx_/(d*Du_ - (Clx_ - Crx_))
	/// @return delta Y, or DBL_MAX if d-(Clx-Crx) == 0
	double getZ(double d) const;
	/// @brief compute disparity given Z.
	/// Symbolically, ((Clx_-Crx_) + Fx_*Tx_/Z)/Du_.
	/// @return DBL_MAX if Z is zero
	double getDisparity(double Z) const;

	/// @brief converts 3d points from Cartesian space to the image plane of the
	/// left camera.
	void cartToLeftCam(
	    /// 3d points of either Nx1 3-channel or Nx3 1-channel.
	    const CvMat* XYZs,
	    /// 2d points of either Nx1 2-channel or Nx2 1-channel.
	    CvMat* uvs) const;

  /// @brief Convert disparity coordinate into pixel location in left camera image.
	/// @return pixel location in rectified left camera image.
  static inline CvPoint dispToLeftCam(
      /// coordinate in disparity coordinates
      const CvPoint3D64f& dispCoord)  {
    return cvPoint(
        (int)std::floor(dispCoord.x + .5),
        (int)std::floor(dispCoord.y + .5)
    );
  }
  /// @brief Convert disparity coordinate into pixel location in left camera image.
  /// @return pixel location in rectified right camera image.
  static inline CvPoint dispToRightCam(
      /// coordinate in disparity coordinates
      const CvPoint3D64f& dispCoord)  {
    return cvPoint(
        (int)std::floor(dispCoord.x + .5),
        (int)std::floor(dispCoord.y - dispCoord.z + .5)
    );
  }


	/// \brief This routine is used to display a single channel floating point depth image.
	/// It inverts the depth so that brightest points are closest.
	/// @param Iz  One Channel, float image.  Depth image (in mm).  If Iz=NULL, shut off display: e.g. just call member dspl_depth_image(); to turn off
	///            Just call the function with an image to display it.  Size of the image can change each frame.
	/// @param Zmin, min depth to dispaly in meters. Zero values for thise => compute from image,
  /// @param Zmax  max depth to display in meters.  Zero values for thise => compute from image,
	void dspl_depth_image(IplImage *Iz=NULL, double Zmin=0.0, double Zmax = 0.0);

	typedef enum {
	  NO_POST_PROCESS,
	  POLYGONES,
	  CONVEX_HULLS,
	} PostProcessOptions;
	/// \brief compute a depth mask according to the minZ and maxZ.
	/// This method generates a mask with pixel locations where depth in 3d are
	/// within the specified range.
	/// If post_process_opt = NO_POST_PROCESS, no processing is done
	/// If post_process_opt = POLYGONES, small groups of connected components in the mask
	/// are removed. The rest are merge into larger connected components, and
	/// grow a certain size.
	/// If post_process_opt = CONVEX_HULLS, small groups of connected components in the mask
	/// are removed. The rest are merge into larger connected components, and
	/// further replaced by the convex hulls of them.
	/// This method has only been tested with raw disparity (U8 1 channel)
	void getDepthMask(/// disparity image
					const IplImage* dispImg,
					/// pre-allocate image buffer for the depth mask
					IplImage* depthMask,
					/// mininum z (in mm) in mask
					double minZ,
					/// max z (in mm) in mask
					double maxZ,
					/// post processing options.
					PostProcessOptions post_process_opt = POLYGONES
	) const;

	void getParams(double* Fx, double* Fy, double* Tx, double* Clx, double* Crx,
	    double* Cy, double* dispUnitScale) const;

	static const double DefaultDispUnitScale=1.0;

protected:
  static void constructMat3DToScreen(double Fx, double Fy, double Tx, double Cx, double Cy,
        CvMat& mat);
  void constructProjectionMatrices();
  /// temporary function, shall be replace by one from cvaux when opencv_latest get updated
  void connectedComponents(IplImage *mask, int poly1_hull0,
         IplConvKernel* openKernel,
         IplConvKernel* closeKernel,
         float perimScale, int *num, CvRect *bbs, CvPoint *centers) const;

  /*!
   * \brief class  member initialization.
   * Only called by constructors. Shall be replaced by delegating constructor
   * when available.
   */
  void init();

  bool   parameterized_;
	double Fx_, Fy_;   //< focal lengths of the rectified image (in pixels)
	double Tx_;        //< translation of right camera relative to left camera
	double Clx_, Crx_, Cy_;  //< the optical centers in pixels left: (Clx_, Cy_) right: (Crx_, Cy_)
	/// the unit of the value in a disparity map. e.g. 1/4 of a pixel.
	double Du_;

	// a set of "default parameters" copied from an example file

	double matdata_cart_to_screen_left_[3*4];
	double matdata_cart_to_screen_right_[3*4];
	double matdata_cart_to_disp_[4*4];
	double matdata_disp_to_cart_[4*4];
	CvMat  mat_cart_to_screen_left_;  //< projection matrix from Cartesian coordinate to the screen image of the left  camera
	CvMat  mat_cart_to_screen_right_; //< projection matrix from Cartesian coordinate to the screen image of the right camera
	CvMat  mat_cart_to_disp_;  //< projection matrix from Cartesian coordinate to the disparity space
	CvMat  mat_disp_to_cart_;  //< projection matrix from disparity space to Cartesian space
  IplImage  *Iz8U_;  //Holds depth image to display for debug purposes

  mutable CvMemStorage*	mem_storage_;
  /// approx threshold - the bigger it is, the simpler is the boundary
  static const int CVCONTOUR_APPROX_LEVEL = 2;
  /// how many iterations of erosion and/or dilation
  static const int CVCLOSE_ITR = 1;


};

#endif /*WGSTEREOCAMMODEL_H_*/
