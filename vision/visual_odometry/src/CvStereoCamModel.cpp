#include "CvStereoCamModel.h"

#include <float.h>
#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>
#include "CvMatUtils.h"
#include <vector>     //Gary added for collecting valid x,y,d points


CvStereoCamModel::CvStereoCamModel(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
    this->constructProjectionMatrices();
}

CvStereoCamModel::CvStereoCamModel(CvStereoCamParams camParams):
    Parent(camParams),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
	constructProjectionMatrices();
}


CvStereoCamModel::CvStereoCamModel():
    Parent(),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart))
{
	constructProjectionMatrices();
}


CvStereoCamModel::~CvStereoCamModel()
{
}

void CvStereoCamModel::constructMat3DToScreen(double Fx, double Fy, double Tx,
		double Cx, double Cy, CvMat& mat){
    double data [] = {
        Fx, 0, Cx, -Fx*Tx,
        0, Fy, Cy, 0,
        0,  0,  1, 0
    };

    CvMat _P = cvMat(3, 4, CV_64FC1, data);

    cvCopy(&_P, &mat);
}

bool CvStereoCamModel::constructProjectionMatrices(){
    bool status = true;
    constructMat3DToScreen(mFx, mFy, mClx, mCy, mTx, mMatCartToScreenLeft);
    constructMat3DToScreen(mFx, mFy, mCrx, mCy, mTx, mMatCartToScreenRight);


    // construct the matrix that maps from Cartesian coordinates to
    // disparity coordinates
    double data [] = {
        mFx,   0,    mClx,        0,
        0,     mFy,  mCy,         0,
        0,     0,    mClx-mCrx,   mFx*mTx,
        0,     0,    1,           0
    };
    CvMat _P = cvMat(4, 4, CV_64FC1, data);
    cvCopy(&_P, &mMatCartToDisp);

 	// construct the matrix that maps from disparity coordinates to
	// Cartesian coordinates
#if 1
    // the following is computationally cleaner and may introduce less error
	double Q[] =
	{
		1,         0,          0,               -mClx,
		0,   mFx/mFy,          0,        -mCy*mFx/mFy,
		0,         0,          0,                 mFx,
		0,         0,      1/mTx,     -(mClx-mCrx)/mTx
	};
#else
	// the following form is more symmetrical, and goes along with
	// mCartToDisp
	double Q[] =
	{
		1./mFx,    0,          0,           -mClx/mFx,
		0,   	1./mFy,        0,           -mCy/mFy,
		0,         0,          0,                1,
		0,         0,      1./(mTx*mFx),    -(mClx-mCrx)/(mFx*mTx)
	};
#endif

	CvMat _Q = cvMat(4, 4, CV_64F, Q);
	cvCopy(&_Q, &mMatDispToCart);

	return status;
}

bool CvStereoCamModel::setCameraParams(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy){
    bool status = true;

    mFx  = Fx;
    mFy  = Fy;
    mClx = Clx;
    mCrx = Crx;
    mCy  = Cy;
    mTx  = Tx;

    status = this->constructProjectionMatrices();
    return status;
}

bool CvStereoCamModel::setCameraParams(const CvStereoCamParams& params) {
  double Fx, Fy, Tx, Clx, Crx, Cy;
  params.getParams(Fx, Fy, Tx, Clx, Crx, Cy);
  return setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
}

bool CvStereoCamModel::reprojection(const CvMat *uvds, CvMat *XYZs) const {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return dispToCart(*uvds, *XYZs);
}

bool CvStereoCamModel::dispToCart(const CvMat& uvds, CvMat & XYZs) const {
	bool status = true;

	CvMat uvds0;
	CvMat XYZs0;
	cvReshape(&uvds, &uvds0, 3, 0);
	cvReshape(&XYZs, &XYZs0, 3, 0);
	cvPerspectiveTransform(&uvds0, &XYZs0, &mMatDispToCart);
	return status;
}

//Id has to be  16SC1, Ixyz has to be 32F
bool CvStereoCamModel::dispToCart(const IplImage *Id, IplImage *Ixyz) const {
	bool status = true;
	cvReprojectImageTo3D(Id, Ixyz,&mMatDispToCart);
	return status;
}

//Id accepts ROI which then applies to the other images
bool CvStereoCamModel::disp8UToCart32F(const IplImage *Id, float ZnearMM, float ZfarMM, IplImage *Iz, IplImage *Ix, IplImage *Iy) const {
	bool status = true;
	//CHECK AND SET UP
	if((!Id)||(!Iz)) return false; //Call with allocated images
	int w = Id->width, h = Id->height;
	if((w != Iz->width)||(h != Iz->height)) return false;        //Image dimensions have to match oi vey, haven't checked Ix, Iy
	if((Id->nChannels != 1)||(Iz->nChannels != 1)) return false; //All one channel
	if(ZnearMM < 0.0) ZnearMM = 0.0; //Adjusting for illegal values of depth to search in MM
	if(ZfarMM < ZnearMM) ZfarMM = ZnearMM; 
	double dNear_dFar[6];
	CvMat dn_f;
	dNear_dFar[0] = w/2; dNear_dFar[1] = h/2; dNear_dFar[2] = ZnearMM;
	dNear_dFar[3] = w/2; dNear_dFar[4] = h/2; dNear_dFar[5] = ZfarMM;
	cvInitMatHeader(&dn_f, 2, 1, CV_64FC3, dNear_dFar); //Turn our points into a 1 row by 3 channel array
	cvPerspectiveTransform(&dn_f, &dn_f, &mMatCartToDisp); //Convert depth to disparity thresholds
	int dNear = (int)(dNear_dFar[2]+0.5);
	int dFar = (int)(dNear_dFar[5] + 0.5);
//	printf("dNear=%d, dFar=%d\n",dNear,dFar);
	CvRect roi = cvGetImageROI(Id);
	int ws = Id->widthStep;
	int Id_jumpby = ws - roi.width;                           //This is to step the pointer over possible ragged withSteps
	unsigned char *Idptr = (unsigned char *)(Id->imageData + roi.y*ws + roi.x);
	unsigned char disp8U; 
	int goodpoints = 0;

	//FILL POINTS
	vector<double> pts;  //Collect good points here
	vector<int> xy;      //Collect original x,y's here
	pts.reserve(roi.width*roi.height*3); //Allocate all the space we'll need in one shot
	xy.reserve(roi.width*roi.height*3);
	int x,y,i;
	for(y=0; y<roi.height; ++y) {
		for(x=0; x<roi.width; ++x) {
			disp8U = *Idptr++;
			if((dNear > disp8U) && (disp8U > dFar)) { //NOTE: Zero is just not allowed. Don't go there, Greek chaos.
				pts.push_back((double)x);
				pts.push_back((double)y);
				pts.push_back((double)(((double)disp8U)/4.0));
				xy.push_back(x);
				xy.push_back(y);
				++goodpoints;
			}
		}
		Idptr += Id_jumpby;                                    //Step over unused widthStep if any (mostly this is zero)
	}
	cvSetZero(Iz); //Start with a clean image
	if(!goodpoints) return status;
	//SET UP FOR PERSPECTIVE TRANSFORM
	CvMat d2c; //Disparity to Cart header holder
	cvInitMatHeader(&d2c, goodpoints, 1, CV_64FC3, &pts[0]); //We now have a CvMat "view" into vector pts

	//DO PERSPECTIVE TRANSFORM
	cvPerspectiveTransform(&d2c, &d2c, &mMatCartToDisp);  //Note that cvPerspectiveTransform can be done in place!

	//PUT THE POINTS OUT IN THE X,Y & Z IMAGES
	float *Izptr = (float *)Iz->imageData, *Ixptr = (float *)Ix->imageData, *Iyptr = (float *)Iy->imageData; //pt to image data
	vector<double>::iterator ptspos; ptspos = pts.begin(); //Set up iterators to our data
	vector<int>::iterator xypos; xypos = xy.begin();
	int offset;
	for(i = 0; i< goodpoints; ++i) {
		x = *xypos; ++xypos;  //x in grid
		y = *xypos; ++xypos;  //y in grid
		offset = y*w + x;     //pointer offset
		*(Ixptr + offset) = (float)*ptspos; ++ptspos; //X in camera cords
		*(Iyptr + offset) = (float)*ptspos; ++ptspos; //Y in camera cords
		*(Izptr + offset) = (float)*ptspos; ++ptspos; //Z in camera cords
	}
	//Done	
	return status;
}


bool CvStereoCamModel::projection(const CvMat *XYZs, CvMat *uvds) const {
	if (uvds == NULL || XYZs == NULL) {
		return false;
	}
	return cartToDisp(*XYZs, *uvds);
}

bool CvStereoCamModel::cartToDisp(const CvMat& XYZs, CvMat& uvds) const {
	bool status = true;
	CvMat xyzs0;
	CvMat uvds0;
	cvReshape(&XYZs, &xyzs0, 3, 0);
	cvReshape(&uvds, &uvds0, 3, 0);
	cvPerspectiveTransform(&xyzs0, &uvds0, &mMatCartToDisp);
	return status;
}

double  CvStereoCamModel::getDeltaU(double deltaX, double Z) const {
  if (Z == 0) {
    return DBL_MAX;
  }

  return mFx*deltaX/Z;
}

double  CvStereoCamModel::getDeltaX(double deltaU, double d) const {
  double dn = (d - (mClx -mCrx));
  if (dn==0) {
    return 0.;
  }
  return deltaU * mTx/dn;
}
double  CvStereoCamModel::getDeltaV(double deltaY, double Z) const {
  if (Z == 0) {
    return DBL_MAX;
  }

  return mFy*deltaY/Z;
}

double  CvStereoCamModel::getDeltaY(double deltaV, double d) const {
  double dn = (d - (mClx -mCrx))*mFy;
  if (dn==0) {
    return 0.;
  }
  return deltaV * mTx *mFx /dn;
}

double CvStereoCamModel::getZ(double d) const {
  double dn = (d - (mClx - mCrx));
  if (dn == 0) {
    return DBL_MAX;
  }

  return mFx*mTx/dn;
}

double CvStereoCamModel::getDisparity(double Z) const {
  if (Z==0) {
    return  DBL_MAX;
  }
  
  return (mClx-mCrx) + mFx*mTx/Z;
}

