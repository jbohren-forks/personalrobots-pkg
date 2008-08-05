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
    bool setCameraParameters(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy);
    bool convert3DToDisparitySpace(CvMat* src, CvMat* dst);
protected:
    static CvMat* constructMat3DToScreen(double Fx, double Fy, double Tx, double Cx, double Cy);
    bool constructProjectionMatrices();
    
    // a set of "default parameters" copied from an example file
    
    CvMat* mMat3DToScreenLeft;  // projection matrix from 3D coordinate to the screen image of the left  camera
    CvMat* mMat3DToScreenRight; // projection matrix from 3D coordinate to the screen image of the right camera
    CvMat* mMat3DToDisp;  // projection matrix from 3D coordinate to the parity space    
};

#endif /*WGSTEREOCAMMODEL_H_*/
