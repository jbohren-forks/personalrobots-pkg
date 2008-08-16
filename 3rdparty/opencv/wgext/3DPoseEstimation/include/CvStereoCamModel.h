#ifndef WGSTEREOCAMMODEL_H_
#define WGSTEREOCAMMODEL_H_

#include "CvStereoCamParams.h"
#include <opencv/cxtypes.h>

class CvStereoCamModel : public CvStereoCamParams
{
public:
    typedef CvStereoCamParams Parent;
    /**
     *  Fx  - focal length in x direction of the rectified image in pixels.
     *  Fy  - focal length in y direction of the rectified image in pixels.
     *  Tx  - Translatation in x direction from the left camera to the right camera.
     *  Clx - x coordinate of the optical center of the left  camera
     *  Crx - x coordinate of the optical center of the right camera
     *  Cy  - y coordinate of the optical center of both left and right camera
     */
    CvStereoCamModel(double Fx, double Fy, double Tx, double Clx=0.0, double Crx=0.0, double Cy=0.0);
    CvStereoCamModel(CvStereoCamParams camParams);
    CvStereoCamModel();
    virtual ~CvStereoCamModel();
    /**
     *  Fx  - focal length in x direction of the rectified image in pixels.
     *  Fy  - focal length in y direction of the rectified image in pixels.
     *  Tx  - Translatation in x direction from the left camera to the right camera.
     *  Clx - x coordinate of the optical center of the left  camera
     *  Crx - x coordinate of the optical center of the right camera
     *  Cy  - y coordinate of the optical center of both left and right camera
     */
    bool setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy);
#if 0
    bool convert3DToDisparitySpace(CvMat* src, CvMat* dst);
#endif
	bool projection(CvMat *XYZs, CvMat *uvds);
	bool reprojection(CvMat *uvds, CvMat *XYZs);

protected:
    static void constructMat3DToScreen(double Fx, double Fy, double Tx, double Cx, double Cy,
    		CvMat& mat);
    bool constructProjectionMatrices();

    // a set of "default parameters" copied from an example file

    double _mMatCartToScreenLeft[3*4];
    double _mMatCartToScreenRight[3*4];
    double _mMatCartToDisp[4*4];
    double _mMatDispToCart[4*4];
    CvMat  mMatCartToScreenLeft;  // projection matrix from Cartesian coordinate to the screen image of the left  camera
    CvMat  mMatCartToScreenRight; // projection matrix from Cartesian coordinate to the screen image of the right camera
    CvMat  mMatCartToDisp;  // projection matrix from Cartesian coordinate to the disparity space
    CvMat  mMatDispToCart;  // projection matrix from disparity space to Cartesian space
};

#endif /*WGSTEREOCAMMODEL_H_*/
