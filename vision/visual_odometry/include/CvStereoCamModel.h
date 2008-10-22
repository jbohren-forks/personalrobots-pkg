#ifndef WGSTEREOCAMMODEL_H_
#define WGSTEREOCAMMODEL_H_

#include "CvStereoCamParams.h"
#include <opencv/cxtypes.h>

/**
 * Stereo camera model, including parameters and transformation derived from them.
 */
class CvStereoCamModel : public CvStereoCamParams
{
public:
  typedef CvStereoCamParams Parent;
  /**
   *  @param Fx  - focal length in x direction of the rectified image in pixels.
   *  @param Fy  - focal length in y direction of the rectified image in pixels.
   *  @param Tx  - Translation in x direction from the left camera to the right camera.
   *  @param Clx - x coordinate of the optical center of the left  camera
   *  @param Crx - x coordinate of the optical center of the right camera
   *  @param Cy  - y coordinate of the optical center of both left and right camera
   */
  CvStereoCamModel(double Fx, double Fy, double Tx, double Clx=0.0, double Crx=0.0, double Cy=0.0);
  CvStereoCamModel(CvStereoCamParams camParams);
  CvStereoCamModel();
  virtual ~CvStereoCamModel();
  /**
   *  @param Fx  - focal length in x direction of the rectified image in pixels.
   *  @param Fy  - focal length in y direction of the rectified image in pixels.
   *  @param Tx  - Translation in x direction from the left camera to the right camera.
   *  @param Clx - x coordinate of the optical center of the left  camera
   *  @param Crx - x coordinate of the optical center of the right camera
   *  @param Cy  - y coordinate of the optical center of both left and right camera
   */
  bool setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy);
  bool setCameraParams(const CvStereoCamParams& params);

  /// Convert 3D points from Cartesian coordinates to disparity coordinates.
	bool projection(
      /// (Input) 3D points stored in rows, in Cartesian coordinates.
	    const CvMat *XYZs,
      /// (Output) 3D points stored in rows, in disparity coordinates.
	    CvMat *uvds) const;
  /// Convert 3D points from disparity coordinates to Cartesian coordinates.
	bool reprojection(
      /// (Input) 3D points stored in rows, in disparity coordinates.
	    const CvMat *uvds,
      /// (Output) 3D points stored in rows, in Cartesian coordinates.
	    CvMat *XYZs) const;

  /// Convert 3D points from Cartesian coordinates to disparity coordinates.
	bool dispToCart(
      /// (Input) 3D points stored in rows, in disparity coordinates.
	    const CvMat& uvds,
      /// (Output) 3D points stored in rows, in Cartesian coordinates.
	    CvMat& XYZs) const;

	// OR ALTERNATIVE INTERFACE WITH IMAGES (Id has to be  16SC1, Ixyz has to be 32F)
	bool dispToCart(
    /// Id has to be 16SC1
    const IplImage *Id, 
    /// Ixyz has to be 32F
		IplImage *Ixyz) const;


  /// Convert 3D points from disparity coordinates to Cartesian coordinates.
	bool cartToDisp(
      /// (Input) 3D points stored in rows, in Cartesian coordinates.
	    const CvMat& XYZs,
      /// (Output) 3D points stored in rows, in disparity coordinates.
	    CvMat& uvds) const;

	/// Get references to the projection matrices. Used mostly for
	/// debugging.
	void getProjectionMatrices(CvMat& cartToDisp, CvMat& dispToCart) {
	  cartToDisp = mMatCartToDisp;
	  dispToCart = mMatDispToCart;
	}

	/// compute delta u, given Z and delta X in Cartesian space
	double getDeltaU(double deltaX, double Z) const;
	/// compute delta X, given disparity and delta u in disparity space
	double getDeltaX(double deltaU, double d) const;
	/// compute delta v, given Z and delta Y in Cartesian space
	double getDeltaV(double deltaY, double Z) const;
	/// compute delta Y, given disparity and delta v in disparity space
	double getDeltaY(double deltaV, double d) const;

protected:
    static void constructMat3DToScreen(double Fx, double Fy, double Tx, double Cx, double Cy,
    		CvMat& mat);
    bool constructProjectionMatrices();

    // a set of "default parameters" copied from an example file

    double _mMatCartToScreenLeft[3*4];
    double _mMatCartToScreenRight[3*4];
    double _mMatCartToDisp[4*4];
    double _mMatDispToCart[4*4];
    CvMat  mMatCartToScreenLeft;  //< projection matrix from Cartesian coordinate to the screen image of the left  camera
    CvMat  mMatCartToScreenRight; //< projection matrix from Cartesian coordinate to the screen image of the right camera
    CvMat  mMatCartToDisp;  //< projection matrix from Cartesian coordinate to the disparity space
    CvMat  mMatDispToCart;  //< projection matrix from disparity space to Cartesian space
};

#endif /*WGSTEREOCAMMODEL_H_*/
