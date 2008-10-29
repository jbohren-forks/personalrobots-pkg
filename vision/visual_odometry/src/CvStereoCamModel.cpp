#include "CvStereoCamModel.h"

#include <float.h>
#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>
#include "CvMatUtils.h"
#include <vector>     //Gary added for collecting valid x,y,d points

static IplConvKernel* DilateKernel = cvCreateStructuringElementEx(15, 15, 7, 7, CV_SHAPE_RECT);
static IplConvKernel* OpenKernel   = cvCreateStructuringElementEx(3, 3, 1, 1,   CV_SHAPE_RECT);
static IplConvKernel* CloseKernel  = cvCreateStructuringElementEx(7, 7, 3, 3,   CV_SHAPE_RECT);

CvStereoCamModel::CvStereoCamModel(double Fx, double Fy, double Tx,
      double Clx, double Crx, double Cy, double dispUnitScale):
        mFx(Fx), mFy(Fy), mTx(Tx), mClx(Clx), mCrx(Crx), mCy(Cy),
        mDu(dispUnitScale),
        mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
        mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
        mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
        mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart)),
        Iz8U(NULL)
{
    this->constructProjectionMatrices();
}

CvStereoCamModel::CvStereoCamModel(const CvStereoCamModel& camModel):
  mFx(camModel.mFx), mFy(camModel.mFy), mTx(camModel.mTx),
  mClx(camModel.mClx), mCrx(camModel.mCrx), mCy(camModel.mCy),
  mDu(camModel.mDu),
  mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
  mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
  mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
  mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart)),
  Iz8U(NULL) {
  this->constructProjectionMatrices();
}

#if 0
CvStereoCamModel::CvStereoCamModel(CvStereoCamParams_Deprecated camParams):
    Parent(camParams),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart)),
    Iz8U(NULL)
{
	constructProjectionMatrices();
}


CvStereoCamModel::CvStereoCamModel():
    Parent(),
    mMatCartToScreenLeft(cvMat(3, 4, CV_64FC1, _mMatCartToScreenLeft)),
    mMatCartToScreenRight(cvMat(3, 4, CV_64FC1, _mMatCartToScreenRight)),
    mMatCartToDisp(cvMat(4, 4, CV_64FC1, _mMatCartToDisp)),
    mMatDispToCart(cvMat(4, 4, CV_64FC1, _mMatDispToCart)),
    Iz8U(NULL)
{
	constructProjectionMatrices();
}
#endif

CvStereoCamModel::~CvStereoCamModel()
{
	if(Iz8U) {
		cvReleaseImage(&Iz8U);
		cvDestroyWindow("Depth");
	}

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
        mFx,   0,    mClx,              0,
        0,     mFy,  mCy,               0,
        0,     0,    (mClx-mCrx)/mDu,   mFx*mTx/mDu,
        0,     0,    1,                 0
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
		0,         0,      mDu/mTx,     -(mClx-mCrx)/mTx
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

bool CvStereoCamModel::setCameraParams(double Fx, double Fy, double Tx,
    double Clx, double Crx, double Cy, double dispUnitScale){
    bool status = true;

    mFx  = Fx;
    mFy  = Fy;
    mClx = Clx;
    mCrx = Crx;
    mCy  = Cy;
    mTx  = Tx;
    mDu  = dispUnitScale;

    status = this->constructProjectionMatrices();
    return status;
}

bool CvStereoCamModel::setCameraParams(const CvStereoCamModel& cm) {
  return setCameraParams(cm.mFx, cm.mFy, cm.mTx, cm.mClx, cm.mCrx, cm.mCy, cm.mDu);
}

void CvStereoCamModel::getParams(double& Fx, double &Fy, double& Tx, double& Clx, double& Crx,
    double& Cy, double& dispUnitScale) const {
  Fx  = mFx;
  Fy  = mFy;
  Tx  = mTx;
  Clx = mClx;
  Crx = mCrx;
  Cy  = mCy;
  dispUnitScale = mDu;
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
#if 0
	double dNear_dFar[6];
	CvMat dn_f;
	dNear_dFar[0] = w/2; dNear_dFar[1] = h/2; dNear_dFar[2] = ZnearMM;
	dNear_dFar[3] = w/2; dNear_dFar[4] = h/2; dNear_dFar[5] = ZfarMM;
	cvInitMatHeader(&dn_f, 2, 1, CV_64FC3, dNear_dFar); //Turn our points into a 1 row by 3 channel array
	cvPerspectiveTransform(&dn_f, &dn_f, &mMatCartToDisp); //Convert depth to disparity thresholds
	int dNear = (int)(dNear_dFar[2]+0.5);
	int dFar = (int)(dNear_dFar[5] + 0.5);
#else
	int dNear = (int)(getDisparity(ZfarMM)  + .5);
	int dFar  = (int)(getDisparity(ZnearMM) + .5);
#endif
//	printf("dNear=%d, dFar=%d\n",dNear,dFar);
	CvRect roi = cvGetImageROI(Id);
	int ws = Id->widthStep, roiw = roi.width, roih = roi.height, roix = roi.x, roiy = roi.y;
	int Id_jumpby = ws - roiw;                      //This is to step the pointer over possible ragged withSteps
	unsigned char *Idptr = (unsigned char *)(Id->imageData + roiy*ws + roix);
	unsigned char disp8U;
	int goodpoints = 0;

	//FILL POINTS
	vector<double> pts;  //Collect good points here
	vector<int> xy;      //Collect original x,y's here
	pts.reserve(roiw*roih*3); //Allocate all the space we'll need in one shot
	xy.reserve(roiw*roih*3);
	int x,y,i;
	for(y=roiy; y<roih+roiy; ++y) {
		for(x=roix; x<roiw+roix; ++x) {
			disp8U = *Idptr++;
			if((dNear > disp8U) && (disp8U > dFar)) { //NOTE: Zero is just not allowed. Don't go there, Greek chaos.
				pts.push_back((double)x);
				pts.push_back((double)y);
				pts.push_back((double)disp8U);
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
  double dn = (d*mDu - (mClx -mCrx));
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
  double dn = (d*mDu - (mClx -mCrx))*mFy;
  if (dn==0) {
    return 0.;
  }
  return deltaV * mTx *mFx /dn;
}

double CvStereoCamModel::getZ(double d) const {
  double dn = (d*mDu - (mClx - mCrx));
  if (dn == 0) {
    return DBL_MAX;
  }

  return mFx*mTx/dn;
}

double CvStereoCamModel::getDisparity(double Z) const {
  if (Z==0) {
    return  DBL_MAX;
  }

  return ((mClx-mCrx) + mFx*mTx/Z)/mDu;
}

//Iz is single channel 32F.  Zero values are not displayed. Passing a null image turns this off
void CvStereoCamModel::dspl_depth_image(IplImage *Iz, double Zmin, double Zmax)
{
	assert(Iz->nChannels == 1);
	//SHUT DOWN WINDOW IF Iz is NULL
	if(!Iz) {
		if(Iz8U) {
			cvDestroyWindow("Depth");
			cvReleaseImage(&Iz8U);
		}
		return;
	}
	//CHECK IF WE NEED TO INIT
	if((!Iz8U) || (Iz8U->width != Iz->width) || (Iz8U->height != Iz->height)) {
		if(Iz8U) {
			cvReleaseImage(&Iz8U);
			cvDestroyWindow("Depth");
		}
		Iz8U = cvCreateImage(cvSize(Iz->width,Iz->height),IPL_DEPTH_8U,1);
		cvNamedWindow("Depth",0);
	}
	//SET UP PARAMETERS
	int w = Iz->width, h = Iz->height, ws = Iz8U->widthStep;
	int jumpby = ws - w;
	Zmin *= 1000.0; Zmax *= 1000.0; //depth is in meters, depth map is in milimeters
	bool mZ = false, MZ = false;    // if Zmin or Zmax == 0, then compute their values from min and max on depth map
	if(Zmin == 0.0) { mZ =  true; Zmin = 99999999.0;}
	if(Zmax == 0.0) MZ = true;
	float *fptr = (float *)(Iz->imageData),fv;
	if(mZ || MZ){
		for(int i = 0; i<w*h; ++i) {
			fv = *fptr++;
			if(fv <= 0) continue;
			if((mZ) && (Zmin > fv)) Zmin = fv;
			if((MZ) && (Zmax < fv)) Zmax = fv;
		}
	}
    if(Zmax <= Zmin) Zmax += 1.0; //Avoid max <= min
    double scaleit = 255.0 / (Zmax - Zmin);
	unsigned char *cptr = (unsigned char *)(Iz8U->imageData);
	fptr = (float *)(Iz->imageData);

	//LOOP
	for(int y=0; y<h; ++y){
		for(int x=0; x<w; ++x) {
			fv = *fptr++;
			if((fv <= 0.0)||(fv > Zmax)){
				*cptr++ = 0;
			}
			else if(fv < Zmin) {
				*cptr++ = 255;
			}
			else {
				*cptr++ = (unsigned char)((double)255.0 - scaleit*((double)fv - Zmin));	  //Invert so that closer objects are brighter
			}
		}
		cptr += jumpby;
	}
	//SHOW THE PRETTY DEPTH IMAGE
	cvShowImage("Depth",Iz8U);
}


/// compute a depth mask according to the minZ and maxZ
void CvStereoCamModel::getDepthMask(/// disparity image
				    const IplImage* dispImg,
				    /// pre-allocate image buffer for the depth mask
				    IplImage* depthMask,
				    /// mininum z in mask
				    double minZ,
				    /// max z in mask
				    double maxZ){
    double maxDisp = getDisparity(minZ);
    double minDisp = getDisparity(maxZ);

    printf("range mask [%f, %f] => [%f, %f]\n", minZ, maxZ, minDisp, maxDisp);

    // fill in the mask according to disparity or depth
    cvInRangeS(dispImg, cvScalar(minDisp), cvScalar(maxDisp), depthMask);

#if 1
    // two simple morphology operation seem to be good enough. But
    // but connected component analysis provides blob with better shape
    cvMorphologyEx(depthMask, depthMask, NULL, OpenKernel, CV_MOP_OPEN, 1);
    //cvMorphologyEx(depthMask, depthMask, NULL, DilateKernel, CV_MOP_CLOSE, 1);
    cvDilate(depthMask, depthMask, DilateKernel, 1);
#else

    const float perimScale = 16;
    const int maxNumBBoxes = 25;
    CvRect bboxes[maxNumBBoxes];
    int numCComp = maxNumBBoxes;
    connectedComponents(depthMask, 0,
			OpenKernel,
			CloseKernel,
			perimScale, &numCComp,
			(CvRect *)NULL, // bboxes,
			(CvPoint *)NULL);

    //printf("found %d blobs\n", numCComp);

    cvDilate(depthMask, depthMask, DilateKernel, 1);
#endif
  }
