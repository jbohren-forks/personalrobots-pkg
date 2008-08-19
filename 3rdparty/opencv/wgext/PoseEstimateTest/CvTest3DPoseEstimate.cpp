
#include <iostream>
#include <vector>

#include <opencv/cxcore.h>
#include "CvTest3DPoseEstimate.h"

#include "Cv3DPoseEstimateDisp.h"
#include "Cv3DPoseEstimate.h"
//#include "Cv3DPoseEstimateRef.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

// VTK headers
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily
#include <CvPoseEstErrMeasDisp.h>

using namespace std;

#define GAUSSIANNOISE

#define ShowTemplateMatching 0

#define DEBUG 1

CvTest3DPoseEstimate::CvTest3DPoseEstimate():
	Parent(),
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0)
{
	_init();
};
CvTest3DPoseEstimate::CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy),
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0)
{
	_init();
}

CvTest3DPoseEstimate::~CvTest3DPoseEstimate()
{
}

void CvTest3DPoseEstimate::_init(){
	cvInitMatHeader(&mRot,   3, 3, CV_64FC1, mRotData);
	cvInitMatHeader(&mTrans, 3, 1, CV_64FC1, mTransData);

}

/**
 *  points0  - numPoints x 3
 *  points1  - numPoints x 3
 */
void CvTest3DPoseEstimate::transform(CvMat *points0, CvMat *points1) {
	// create a translated/rotated copy of the 3D points
    // the euler angle of the rotation matrix is given in z-y-x form, aka roll-yaw-pitch, Tait-Bryan angles
	double roll, pitch, yaw;
	pitch = mEulerAngle.x;
	yaw   = mEulerAngle.y;
	roll  = mEulerAngle.z;

#if 0 // ZYX
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::ZYX);
	cout << "Rotation Matrix:= Pitch * Yaw* Roll"<< endl;
#else // XYZ euler angle
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::XYZ);
	cout << "Rotation Matrix:= Roll * Yaw * Pitch: "<< mEulerAngle.x/CV_PI*180. << ","<<mEulerAngle.y/CV_PI*180.<<","<<mEulerAngle.z/CV_PI*180.<< endl;
#endif
#if 1
	CvMatUtils::printMat(&mRot);
#endif

	cvmSet(&mTrans, 0, 0, mTranslation.x);
	cvmSet(&mTrans, 1, 0, mTranslation.y);
	cvmSet(&mTrans, 2, 0, mTranslation.z);

#if 1
	cout << "Translation Matrix: "<< endl;
	CvMatUtils::printMat(&mTrans);
#endif

	// R * transpose(points0) + T
	transform(&mRot, points0, &mTrans, points1);
}

void CvTest3DPoseEstimate::transform(CvMat* R, CvMat *points0, CvMat* T, CvMat *points1) {
	int numPoints = points0->rows;
	CvMat * points1t =  cvCreateMat(3,  numPoints, CV_64FC1);

	CvMat *Temp = cvCreateMat(3, points0->rows, CV_64FC1);
	cvRepeat(T, Temp);
	cvGEMM(R, points0, 1.0, Temp, 1.0, points1t, CV_GEMM_B_T);
	cvTranspose(points1t, points1);

#if 0
	cout << "original points: " << endl;
	CvMatUtils::printMat(points0);
	cout << "transformed points: "<<endl;
	CvMatUtils::printMat(points1);
	cout << "Rot and Trans: "<< endl;
	CvMatUtils::printMat(R);
	CvMatUtils::printMat(T);
#endif
}

double CvTest3DPoseEstimate::randReal(double min, double max) {
	double r = cvRandReal(&mRng);
	r = r*(max-min) + min;
	return r;
}

void CvTest3DPoseEstimate::disturb(const CvMat *xyzs, CvMat* xyzsNoised) {
#ifdef GAUSSIANNOISE
//	CvMat* Noise = cvCreateMat(xyzs->rows, xyzs->cols, xyzs->type);
	double sigma = mDisturbScale/2.0; // ~ 95%
	cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
	cvAdd(xyzs, xyzsNoised, xyzsNoised);
//	cvReleaseMat(&Noise);
#else
	double s = mDisturbScale;
	if (s<=0.0)
		return;
	for (int i=0; i<xyzs->rows; i++){
		cvSetReal2D(xyzs, i, 0, cvGetReal2D(xyzs, i, 0)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 1, cvGetReal2D(xyzs, i, 1)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 2, cvGetReal2D(xyzs, i, 2)+randReal(-s, s));
	}
#endif
}

void CvTest3DPoseEstimate::randomize(CvMat *xyzs, int num, double maxVal){
	// fill in the random number in (-maxVal and maxVal)
	double r;
	for (int i=0; i<num; i++){
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 0, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 1, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 2, r);
	}
}

bool CvTest3DPoseEstimate::test() {
	switch (this->mTestType) {
	case Cartesian:
	case Disparity:
	case CartAndDisp:
		return testPointClouds();
		break;
	case Video:
		return testVideos();
		break;
	default:
		cout << "Unknown test type: "<<  mTestType << endl;
	}
	return false;
}

void CvTest3DPoseEstimate::display3d() {
	//
	// Next we create an instance of vtkConeSource and set some of its
	// properties. The instance of vtkConeSource "cone" is part of a
	// visualization pipeline (it is a source process object); it produces data
	// (output type is vtkPolyData) which other filters may process.
	//
	vtkConeSource *cone = vtkConeSource::New();
	cone->SetHeight( 3.0 );
	cone->SetRadius( 1.0 );
	cone->SetResolution( 10 );

	//
	// In this example we terminate the pipeline with a mapper process object.
	// (Intermediate filters such as vtkShrinkPolyData could be inserted in
	// between the source and the mapper.)  We create an instance of
	// vtkPolyDataMapper to map the polygonal data into graphics primitives. We
	// connect the output of the cone souece to the input of this mapper.
	//
	vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
	coneMapper->SetInputConnection( cone->GetOutputPort() );

	//
	// Create an actor to represent the cone. The actor orchestrates rendering
	// of the mapper's graphics primitives. An actor also refers to properties
	// via a vtkProperty instance, and includes an internal transformation
	// matrix. We set this actor's mapper to be coneMapper which we created
	// above.
	//
	vtkActor *coneActor = vtkActor::New();
	coneActor->SetMapper( coneMapper );

	//
	// Create the Renderer and assign actors to it. A renderer is like a
	// viewport. It is part or all of a window on the screen and it is
	// responsible for drawing the actors it has.  We also set the background
	// color here.
	//
	vtkRenderer *ren1= vtkRenderer::New();
	ren1->AddActor( coneActor );
	ren1->SetBackground( 0.1, 0.2, 0.4 );

	//
	// Finally we create the render window which will show up on the screen.
	// We put our renderer into the render window using AddRenderer. We also
	// set the size to be 300 pixels by 300.
	//
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer( ren1 );
	renWin->SetSize( 640, 480 );

	//
	// The vtkRenderWindowInteractor class watches for events (e.g., keypress,
	// mouse) in the vtkRenderWindow. These events are translated into
	// event invocations that VTK understands (see VTK/Common/vtkCommand.h
	// for all events that VTK processes). Then observers of these VTK
	// events can process them as appropriate.
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	//
	// By default the vtkRenderWindowInteractor instantiates an instance
	// of vtkInteractorStyle. vtkInteractorStyle translates a set of events
	// it observes into operations on the camera, actors, and/or properties
	// in the vtkRenderWindow associated with the vtkRenderWinodwInteractor.
	// Here we specify a particular interactor style.
	vtkInteractorStyleTrackballCamera *style =
		vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	//
	// Unlike the previous scripts where we performed some operations and then
	// exited, here we leave an event loop running. The user can use the mouse
	// and keyboard to perform the operations on the scene according to the
	// current interaction style. When the user presses the "e" key, by default
	// an ExitEvent is invoked by the vtkRenderWindowInteractor which is caught
	// and drops out of the event loop (triggered by the Start() method that
	// follows.
	//
	iren->Initialize();
	iren->Start();

	//
	// Final note: recall that an observers can watch for particular events and
	// take appropriate action. Pressing "u" in the render window causes the
	// vtkRenderWindowInteractor to invoke a UserEvent. This can be caught to
	// popup a GUI, etc. So the Tcl Cone5.tcl example for an idea of how this
	// works.

	//
	// Free up any objects we created. All instances in VTK are deleted by
	// using the Delete() method.
	//
	cone->Delete();
	coneMapper->Delete();
	coneActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();
}

static void MyMouseCallback(int event, int x, int y, int flagsm, void* param){
	CvTest3DPoseEstimate *pe = (CvTest3DPoseEstimate *)param;
	switch(event) {
	case CV_EVENT_LBUTTONDBLCLK:
		if (flagsm & CV_EVENT_FLAG_CTRLKEY) {
			pe->display3d();
		}
		break;
	default:
		;
	}
}

void cvCross(CvArr* img, CvPoint pt, int halfLen, CvScalar color,
        int thickness=1, int line_type=8, int shift=0) {
	CvPoint pt1;
	CvPoint pt2;
	pt1.x = pt.x - halfLen;
	pt2.x = pt.x + halfLen;
	pt1.y = pt2.y = pt.y;
	cvLine(img, pt1, pt2, color, thickness, line_type, shift);
	pt1.x = pt2.x = pt.x;
	pt1.y = pt.y - halfLen;
	pt2.y = pt.y + halfLen;
	cvLine(img, pt1, pt2, color, thickness, line_type, shift);
}

bool CvTest3DPoseEstimate::testVideos() {
	bool status = false;
	int numImages = 1509;
	double ransacInlierthreshold = 5.0;
	int numRansacIterations = 100;
//	int numImages = 1;
	IplImage* leftimg  = 0;
	IplImage* rightimg = 0;
	CvMat * harrisCorners = NULL;
	char leftfilename[PATH_MAX];
	char rightfilename[PATH_MAX];
	char leftCamWithMarks[PATH_MAX];
	char rightCamWithMarks[PATH_MAX];
	char dispMapFilename[PATH_MAX];
	char poseEstFilename[PATH_MAX];


	Cv3DPoseEstimateDisp peDisp;
	// The following parameters are from indoor1/proj.txt
	// note that B (or Tx) is in mm
	this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
	peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);
	peDisp.setInlierErrorThreshold(ransacInlierthreshold);
	peDisp.setNumRansacIterations(numRansacIterations);

	// create a list of windows to display results
	cvNamedWindow("Pose Estimated", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Left  Cam", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Right Cam", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Disparity Map", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("Texture", CV_WINDOW_AUTOSIZE);
#if ShowTemplateMatching==1
	cvNamedWindow("Template", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Neighborhood", CV_WINDOW_AUTOSIZE);
#endif

	cvMoveWindow("Pose Estimated", 0, 0);
	cvMoveWindow("Left  Cam", 650, 0);
	cvMoveWindow("Right Cam", 1300, 0);
	cvMoveWindow("Disparity Map", 650, 530);
//	cvMoveWindow("Texture", 1300, 530);

#if ShowTemplateMatching==1
	cvMoveWindow("Template", 0, 530);
	cvMoveWindow("Neighborhood", 300, 530);
#endif


	cvSetMouseCallback("Left  Cam", MyMouseCallback, (void*)this);


	string dirname = "Data/indoor1";
	string outputDirname = "Output/indoor1";
	const int max_features = 100;
	CvPoint2D32f featuresLastLeft[max_features];
	CvPoint2D32f featuresLeft[max_features];
	CvPoint2D32f featuresRight[max_features];
	int numFeaturesLastLeft=0;
	CvMat *lastDispImg = NULL; //
	IplImage *lastleftimg = NULL;   // original image

	int maxDisp = (int)peDisp.getD(10); // the closest point we care is at least 100 mm away
	cout << "Max disparity is: "<< maxDisp<<endl;

	CvPoseEstErrMeasDisp peErrMeas;
	peErrMeas.setParams(mFx, mFy, mTx, mClx, mCrx, mCy);

	int start = 0;
//	int end   = numImages;
	int end   = 5;
	int numFrames = end - start;

	// keep track of the trajectory of the camera w.r.t to the starting frame
	double _shifts[numFrames*3];
	double _rods[numFrames*3]; // Rodrigues
	CvMat shifts;
	CvMat rods;    // Rodrigues
	shifts = cvMat(numFrames, 3, CV_64FC1, _shifts);
	rods   = cvMat(numFrames, 3, CV_64FC1, _rods);

	// Current transformation w.r.t to the starting frame
	double _transform[16];
	CvMat transform = cvMat(4, 4, CV_64FC1, _transform);
	cvSetIdentity(&transform);
	// current transformation w.r.t. to the current frame
	double _rt[16];
	CvMat rt = cvMat(4, 4, CV_64FC1, _rt);
	double _tempMat[16];
	CvMat tempMat = cvMat(4, 4, CV_64FC1, _tempMat);

	for (int i=start; i<end; i++) {
		sprintf(leftfilename,  "%s/left-%04d.ppm",  dirname.c_str(), i);
		sprintf(rightfilename, "%s/right-%04d.ppm", dirname.c_str(), i);
		cout << "loading "<<leftfilename<<" and "<< rightfilename<< endl;
		leftimg  = cvLoadImage(leftfilename, 0);
		rightimg = cvLoadImage(rightfilename, 0);

		// make a copy of color image for display
		CvMat *leftimgC3  = cvCreateMat(leftimg->height,  leftimg->width,  CV_8UC3);
		CvMat *leftimgC3a = cvCreateMat(leftimg->height,  leftimg->width,  CV_8UC3);
		CvMat *rightimgC3 = cvCreateMat(rightimg->height, rightimg->width, CV_8UC3);
		cvCvtColor(leftimg,  leftimgC3,  CV_GRAY2RGB);
		cvCvtColor(leftimg,  leftimgC3a, CV_GRAY2RGB);
		cvCvtColor(rightimg, rightimgC3, CV_GRAY2RGB);

		//
		// Try Kurt's dense stereo pair
		//
		uint8_t *lim = (uint8_t *)leftimg->imageData;
		uint8_t *rim = (uint8_t *)rightimg->imageData;
		int xim = leftimg->width;
		int yim = leftimg->height;

		// some parameters
		int ftzero = 31;		// max 31 cutoff for prefilter value
		int dlen   = 64;		// 64 disparities
		int corr   = 15;		// correlation window size
		int tthresh = 10;		// texture threshold
		int uthresh = 15;		// uniqueness threshold

		// allocate buffers
		uint8_t* buf  = (uint8_t *)malloc(yim*dlen*(corr+5)); // local storage for the algorithm
		uint8_t* flim = (uint8_t *)malloc(xim*yim); // feature image
		uint8_t* frim = (uint8_t *)malloc(xim*yim); // feature image
		int16_t* disp = (int16_t *)malloc(xim*yim*2); // disparity image
//		int16_t* textImg = (int16_t *)malloc(xim*yim*2); // texture image
		int16_t* textImg = NULL;

		// prefilter
		do_prefilter(lim, flim, xim, yim, ftzero, buf);
		do_prefilter(rim, frim, xim, yim, ftzero, buf);

		// stereo
		do_stereo(flim, frim, disp, textImg, xim, yim,
				ftzero, corr, corr, dlen, tthresh, uthresh, buf);

		CvMat dispImg = cvMat(yim, xim, CV_16SC1, disp);

		//
		// display disparity map
		//
		double minVal, maxVal;
		cvMinMaxLoc(&dispImg, &minVal, &maxVal);
		printf("min, max of dispImg: %f, %f\n", minVal, maxVal);

		uint8_t _dispImgU8C3[yim*xim*3];
//		uint8_t _textImgU8C3[yim*xim*3];
		CvMat dispImgU8C3 = cvMat(yim, xim, CV_8UC3, _dispImgU8C3);
//		CvMat textImgU8C3 = cvMat(yim, xim, CV_8UC3, _textImgU8C3);
		const double gamma = 0.4;  // between 0.0 and 1.0
		for (int v=0; v<yim; v++) {
			for (int u=0; u<xim; u++) {
				int16_t d = disp[v*xim+u];
				if (d < 0) {
					// set it to blue (BGR)
					_dispImgU8C3[(v*xim+u)*3 + 0] = 255;
					_dispImgU8C3[(v*xim+u)*3 + 1] = 0;
					_dispImgU8C3[(v*xim+u)*3 + 2] = 0;
				} else if (d==0) {
					// set it to yellow (BGR)
					_dispImgU8C3[(v*xim+u)*3 + 0] = 0;
					_dispImgU8C3[(v*xim+u)*3 + 1] = 255;
					_dispImgU8C3[(v*xim+u)*3 + 2] = 255;
				} else if (d > maxDisp) {
					// set it to red (BGR)
					_dispImgU8C3[(v*xim+u)*3 + 0] = 0;
					_dispImgU8C3[(v*xim+u)*3 + 1] = 0;
					_dispImgU8C3[(v*xim+u)*3 + 2] = 255;
				} else {
					uint8_t gray = d/(double)maxDisp * 255.;
					gray = (int)(0.5 + 255.0 * pow((double)gray/255.0, gamma));
					_dispImgU8C3[(v*xim+u)*3 + 0] = gray;
					_dispImgU8C3[(v*xim+u)*3 + 1] = gray;
					_dispImgU8C3[(v*xim+u)*3 + 2] = gray;
				}
#if 0
				int16_t text = textImg[v*xim+u];
				if (text<0) {
					_textImgU8C3[(v*xim+u)*3+0] = 255;
					_textImgU8C3[(v*xim+u)*3+1] = 0;
					_textImgU8C3[(v*xim+u)*3+2] = 0;
				} else if (text == 0) {
					_textImgU8C3[(v*xim+u)*3+0] = 255;
					_textImgU8C3[(v*xim+u)*3+1] = 255;
					_textImgU8C3[(v*xim+u)*3+2] = 0;
				} else {
					_textImgU8C3[(v*xim+u)*3+0] = text;
					_textImgU8C3[(v*xim+u)*3+1] = text;
					_textImgU8C3[(v*xim+u)*3+2] = text;
				}
#endif
			}
		}
		printf("min, max of dispImg: %f, %f\n", minVal, maxVal);
		cvShowImage("Disparity Map", &dispImgU8C3);
//		cvShowImage("Texture", &textImgU8C3);
		// end display disparity map


		//
		//  Try cvGoodFeaturesToTrack
		//
//		harrisCorners     = cvCreateMat(rightimg->height, rightimg->width, CV_32FC1);
		CvMat* eig_image  = cvCreateMat(rightimg->height, rightimg->width, CV_32FC1);
		CvMat* temp_image = cvCreateMat(rightimg->height, rightimg->width, CV_32FC1);
		//cvConvert(rightimg, tmp);
//		cvCornerHarris(tmp, harrisCorners, 21);
		double quality_level =  0.01; // what should I use? [0., 1.0]
		double min_distance  = 10.0; // 10 pixels?
		int numFeaturesLeft=max_features;
		int numFeaturesRight=max_features;

		// misc params to cvGoodFeaturesToTrack
//		const CvArr* mask=NULL;
		int8_t _mask[yim*xim];
		for (int v=0; v<yim; v++) {
			for (int u=0; u<xim; u++) {
				int16_t d = disp[v*xim+u];
				if (d>0 and d <=maxDisp) {
					_mask[v*xim+u] = 1;
				} else {
					_mask[v*xim+u] = 0;
				}
			}
		}
		const CvMat mask= cvMat(yim, xim, CV_8SC1, _mask);
		const int block_size=5;  // default is 3
		const int use_harris=1;   // default is 0;
		const double k=0.01; // default is 0.04;
		cvGoodFeaturesToTrack(leftimg, eig_image, temp_image,
				featuresLeft, &numFeaturesLeft,
				quality_level, min_distance, &mask, block_size, use_harris, k);
		cvGoodFeaturesToTrack(rightimg, eig_image, temp_image,
				featuresRight, &numFeaturesRight,
				quality_level, min_distance, &mask, block_size, use_harris, k);

		cout << "Found "<<numFeaturesLeft <<" good features in left  image"<<endl;
		cout << "Found "<<numFeaturesRight<<" good features in right image"<<endl;

		CvScalar red    = CV_RGB(255, 0, 0);
		CvScalar green  = CV_RGB(0, 255, 0);
		CvScalar yellow = CV_RGB(255, 255, 0);
		for (int k=0;k<numFeaturesLeft; k++){
			cvCircle(leftimgC3, cvPoint((int)featuresLeft[k].x, (int)featuresLeft[k].y),
					4, green, 1, CV_AA, 0);
			cvCircle(rightimgC3, cvPoint((int)featuresLeft[k].x, (int)featuresLeft[k].y),
					0, green, 2, CV_AA, 0);
		}

		for (int k=0; k<numFeaturesLastLeft; k++){
			// draw cross instead of circle
			CvPoint pt = cvPoint(featuresLastLeft[k].x, featuresLastLeft[k].y);
			int halfLen = 4;
			cvCross(leftimgC3, pt, halfLen, yellow, 1, CV_AA, 0);
#if 0
			CvPoint pt1;
			CvPoint pt2;
			int halfLen = 4;
			pt1.x = (int)featuresLastLeft[k].x - halfLen;
			pt2.x = (int)featuresLastLeft[k].x + halfLen;
			pt1.y = pt2.y = (int)featuresLastLeft[k].y;
			cvLine(leftimgC3, pt1, pt2, yellow, 1, CV_AA, 0);
			pt1.x = pt2.x = (int)featuresLastLeft[k].x;
			pt1.y = (int)featuresLastLeft[k].y - halfLen;
			pt2.y = (int)featuresLastLeft[k].y + halfLen;
			cvLine(leftimgC3, pt1, pt2, yellow, 1, CV_AA, 0);
#endif
		}

		for (int k=0;k<numFeaturesRight; k++){
			cvCircle(rightimgC3, cvPoint((int)featuresRight[k].x, (int)featuresRight[k].y),
					0, red, 2, CV_AA, 0);
#if 0
			cvCircle(leftimgC3, cvPoint((int)featuresRight[k].x, (int)featuresRight[k].y),
					0, green, 2, CV_AA, 0);
#endif
		}

		//
		// match the good feature points between this iteration and last
		//
		// change from 61 to 101 to accomodate the sudden change around 0915 in
		// the indoor sequence
//		const CvPoint neighborhoodSize = cvPoint(61, 31);
		const CvPoint neighborhoodSize = cvPoint(101, 31);
		const CvPoint templSize        = cvPoint(11, 11);
		cout <<"Going thru "<< numFeaturesLastLeft<< " good feature points in last left image"<<endl;
		int numTrackablePairs=0;
		vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;
		for (int k=0; k<numFeaturesLastLeft; k++) {
			CvPoint2D32f featurePtLastLeft = featuresLastLeft[k];
			CvPoint fPtLastLeft = cvPoint((int)(featurePtLastLeft.x+.5),
					(int)(featurePtLastLeft.y+.5));
			CvScalar s = cvGet2D(lastDispImg, featurePtLastLeft.y, featurePtLastLeft.x);
			double d = s.val[0];
			double d0 = ((int16_t *)(lastDispImg->data.s))[fPtLastLeft.y*xim+fPtLastLeft.x];
			double d1 = CV_MAT_ELEM(*lastDispImg, int16_t, fPtLastLeft.y, fPtLastLeft.x);
			assert( d == d0);
			assert(d0 == d1);
			assert( d>=0);
			CvPoint3D64f ptLast = cvPoint3D64f(featurePtLastLeft.x, featurePtLastLeft.y, d);

			cout << "Feature at "<< featurePtLastLeft.x<<","<<featurePtLastLeft.y<<","<<d<<endl;
			// find the closest (in distance and appearance) feature
			// to it in current feature list
			// In a given neighborhood, if there is at least a good feature
			// in the left cam image,
			// we search for a location in current
			// left cam image is that is closest to this one in appearance
			bool goodFeaturePtInCurrentLeftImg = false;
			vector<CvPoint2D32f> featurePtsInNeighborhood;
			assert(featurePtsInNeighborhood.size()==0);
			for (int l=0; l<numFeaturesLeft; l++) {
				CvPoint2D32f featurePtLeft = featuresLeft[l];
				float dx = featurePtLeft.x - featurePtLastLeft.x;
				float dy = featurePtLeft.y - featurePtLastLeft.y;
				if (fabs(dx)<neighborhoodSize.x && fabs(dy)<neighborhoodSize.y) {
					goodFeaturePtInCurrentLeftImg = true;
					featurePtsInNeighborhood.push_back(featurePtLeft);
					cout << "Good candidate at "<< featurePtLeft.x <<","<< featurePtLeft.y<<endl;
				}
			}
			if (goodFeaturePtInCurrentLeftImg == true) {
				cout <<"Found "<< featurePtsInNeighborhood.size() << " candidates in the neighborhood"<<endl;
				// find the best correlation in the neighborhood
				// make a template center around featurePtLastLeft;
				CvRect rectTempl = cvRect(
						(int)(.5 + featurePtLastLeft.x - templSize.x/2),
						(int)(.5 + featurePtLastLeft.y - templSize.y/2),
						templSize.x, templSize.y
				);
				// check if rectTempl is all within bound
				if (rectTempl.x < 0 || rectTempl.y < 0 ||
						rectTempl.x+rectTempl.width    > lastleftimg->width ||
						rectTempl.y + rectTempl.height > lastleftimg->height ) {
					// (partially) out of bound.
					// skip this
					continue;
				}
				CvRect rectNeighborhood = cvRect(
						(int)(.5 + featurePtLastLeft.x - neighborhoodSize.x/2),
						(int)(.5 + featurePtLastLeft.y - neighborhoodSize.y/2),
						neighborhoodSize.x, neighborhoodSize.y
				);
				if (rectNeighborhood.x < 0 || rectNeighborhood.y < 0 ||
						rectNeighborhood.x + rectNeighborhood.width  > leftimg->width ||
						rectNeighborhood.y + rectNeighborhood.height > leftimg->height ) {
					// (partially) out of bound.
					// skip this
					continue;
				}
				CvMat templ, neighborhood;
				float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
				CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
				cvGetSubRect(lastleftimg, &templ, rectTempl);
				cvGetSubRect(leftimg, &neighborhood, rectNeighborhood);
//				cvGetSubRect(lastleftimg, &neighborhood, rectNeighborhood);  // correlate with itself, shall get 0,0
				cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_SQDIFF );
#if ShowTemplateMatching==1
				cvShowImage("Template",     &templ);
				cvShowImage("Neighborhood", &neighborhood);
				cvWaitKey(0);
#endif
				CvPoint		minloc, maxloc;
				double		minval, maxval;
				cvMinMaxLoc( &res, &minval, &maxval, &minloc, &maxloc, 0 );
				// minloc goes with CV_TM_SQDIFF
				// maxloc goes with CV_TM_CCORR and CV_TM_CCOEFF
				CvPoint     bestloc = minloc;
				bestloc.x += rectTempl.width/2;  // please note that they are integer
				bestloc.y += rectTempl.height/2;
				bestloc.x -= rectNeighborhood.width/2;
				bestloc.y -= rectNeighborhood.height/2;


				cout << "best match offset: "<< bestloc.x <<","<<bestloc.y<<endl;
				bestloc.x += featurePtLastLeft.x;
				bestloc.y += featurePtLastLeft.y;

				CvScalar s = cvGet2D(&dispImg, bestloc.y, bestloc.x);
				double d = s.val[0];
				double d0 = disp[bestloc.y*xim+bestloc.x];
				assert( d == d0);
				if (d<0) {
					cout << "disparity missing: "<<d<<","<< disp[bestloc.y*xim+bestloc.x] <<endl;
					continue;
				}

				CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, d);
				cout << "best match: "<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
				pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
				trackablePairs.push_back(p);
				numTrackablePairs++;
			}
		}
		assert(numTrackablePairs == (int)trackablePairs.size());
		cout << "Num of trackable pairs for pose estimate: "<<numTrackablePairs <<endl;

		if (numTrackablePairs<10) {
			cout << "Too few trackable pairs"<<endl;
		} else {

			//
			// Draw all the trackable pairs
			//
			for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = trackablePairs.begin();
			iter != trackablePairs.end(); iter++) {
				const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
				CvPoint p0 = cvPoint(p.first.x, p.first.y);
				CvPoint p1 = cvPoint(p.second.x, p.second.y);
				int thickness =2;
				cvLine(leftimgC3, p0, p1, red, thickness, CV_AA);

//				cvLine(leftimgC3a, p0, p1, red, thickness, CV_AA);
				//				cvCircle(leftimgC3a, p1, 4, green, 1, CV_AA, 0);
			}


			//
			//  pose estimation given the feature point pairs
			//
			double _uvds0[3*numTrackablePairs];
			double _uvds1[3*numTrackablePairs];
			CvMat uvds0 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds0);
			CvMat uvds1 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds1);

			int iPt=0;
			for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = trackablePairs.begin();
			iter != trackablePairs.end(); iter++) {
				const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
				_uvds0[iPt*3 + 0] = p.first.x;
				_uvds0[iPt*3 + 1] = p.first.y;
				_uvds0[iPt*3 + 2] = p.first.z;

				_uvds1[iPt*3 + 0] = p.second.x;
				_uvds1[iPt*3 + 1] = p.second.y;
				_uvds1[iPt*3 + 2] = p.second.z;
				iPt++;
			}

#if 0
			cout << "trackable pairs"<<endl;
			CvMatUtils::printMat(&uvds0);
			CvMatUtils::printMat(&uvds1);
#endif

			CvMat *inliers0 = NULL;
			CvMat *inliers1 = NULL;
			double _rot[9], _shift[3];
			CvMat rot = cvMat(3, 3, CV_64FC1, _rot);
			CvMat shift = cvMat(3, 1, CV_64FC1, _shift);
			// estimate the transform the observed points from current back to last position
			// it should be equivalent to the transform of the camera frame from
			// last position to current position
			int numInliers =
				peDisp.estimate(&uvds1, &uvds0, &rot, &shift, inliers1, inliers0);

#if DEBUG==1
			// logging the rotation matrix and the shift vector
			cout << "Rot Matrix: "<<endl;
			CvMatUtils::printMat(&rot);
			cout << "Shift Matrix: "<<endl;
			CvMatUtils::printMat(&shift);

			Cv3DPoseEstimate::constructTransform(rot, shift, rt);
			cvCopy(&transform, &tempMat);
			cvMatMul(&tempMat, &rt, &transform);
			CvMat rotGlobal;
			CvMat shiftGlobal;
			cvGetSubRect(&transform, &rotGlobal,   cvRect(0, 0, 3, 3));
			cvGetSubRect(&transform, &shiftGlobal, cvRect(3, 0, 1, 3));
			CvMat rodGlobal2;
			CvMat shiftGlobal2;
			cvGetRow(&rods, &rodGlobal2, i-start);
			cvGetRow(&shifts, &shiftGlobal2, i-start);
			// make a copy
			cvRodrigues2(&rotGlobal, &rodGlobal2);
			cvTranspose(&shiftGlobal,  &shiftGlobal2);


			cout << "num of inliers: "<< numInliers <<endl;

			if (numInliers <= 0 || inliers0 == NULL || inliers1 ==NULL ) {
				cout << "No good estimate for these two poses"<<endl;
			} else {
				// show the inliers

				double _xyzs0[3*numInliers];
				double _xyzs0To1[3*numInliers];
				double _uvds0To1[3*numInliers];
				double _xyzs1[3*numInliers];
				CvMat xyzs0    = cvMat(numInliers, 3, CV_64FC1, _xyzs0);
				CvMat xyzs0To1 = cvMat(numInliers, 3, CV_64FC1, _xyzs0To1);
				CvMat uvds0To1 = cvMat(numInliers, 3, CV_64FC1, _uvds0To1);
				CvMat xyzs1    = cvMat(numInliers, 3, CV_64FC1, _xyzs1);

				peDisp.reprojection(inliers0, &xyzs0);
				peDisp.reprojection(inliers1, &xyzs1);

				// compute the inverse transformation
				double _invRot[9], _invShift[3];
				CvMat invRot   = cvMat(3, 3, CV_64FC1, _invRot);
				CvMat invShift = cvMat(3, 1, CV_64FC1, _invShift);

				cvInvert(&rot, &invRot);
				cvGEMM(&invRot, &shift, -1., NULL, 0., &invShift, 0.0);
				CvMat xyzs0Reshaped;
				CvMat xyzs0To1Reshaped;
				cvReshape(&xyzs0,    &xyzs0Reshaped, 3, 0);
				cvReshape(&xyzs0To1, &xyzs0To1Reshaped, 3, 0);
				cvTransform(&xyzs0Reshaped, &xyzs0To1Reshaped, &invRot, &invShift);

				peDisp.projection(&xyzs0To1, &uvds0To1);

				// draw uvds0To1 on leftimgeC3a
				for (int k=0;k<numInliers;k++) {
					CvPoint pt0To1 = cvPoint((int)(_uvds0To1[k*3+0]+.5), (int)(_uvds0To1[k*3+1] + .5));
					const int halfLen = 4;
					cvCross(leftimgC3a, pt0To1, halfLen, yellow);
					CvPoint pt1 = cvPoint((int)(cvGetReal2D(inliers1, k, 0)+.5), (int)(cvGetReal2D(inliers1, k, 1)+.5));
					cvCircle(leftimgC3a, pt1, 4, green, 1, CV_AA, 0);
					cvLine(leftimgC3a, pt1, pt0To1, red, 1, CV_AA, 0);
				}

				// measure the errors
				peErrMeas.setTransform(rot, shift);
				peErrMeas.measure(*inliers1, *inliers0);
			}
#endif
		}

		//
		//  getting ready for next iteration
		//
		if (leftimgC3a) {
			cvShowImage("Pose Estimated", leftimgC3a);
		}
		cvShowImage("Left  Cam", leftimgC3);
		cvShowImage("Right Cam", rightimgC3);
//		cvShowImage("Harris Corners", harrisCorners);
		//		cvShowImage("Harris Corners", eig_image);

		// save the marked images
#if 1
		sprintf(leftCamWithMarks, "%s/leftCamWithMarks-%04d.png", outputDirname.c_str(), i);
		sprintf(rightCamWithMarks, "%s/rightCamwithMarks-%04d.png", outputDirname.c_str(), i);
		sprintf(dispMapFilename, "%s/dispMap-%04d.png", outputDirname.c_str(), i);
		sprintf(poseEstFilename, "%s/poseEst-%04d.png", outputDirname.c_str(), i);
		cvSaveImage(leftCamWithMarks,  leftimgC3);
		cvSaveImage(rightCamWithMarks, rightimgC3);
		cvSaveImage(dispMapFilename,   &dispImgU8C3);
		cvSaveImage(poseEstFilename,   leftimgC3a);
#endif


		// wait for a while for opencv to draw stuff on screen
//		cvWaitKey(25);  //  milliseconds
		cvWaitKey(0);  //  wait indefinitely

		for (int k=0;k<numFeaturesLeft; k++){
			featuresLastLeft[k] = featuresLeft[k];
		}
		numFeaturesLastLeft = numFeaturesLeft;
		if (lastleftimg)   cvReleaseImage(&lastleftimg);
		if (lastDispImg)   cvReleaseMat(&lastDispImg);
		cvReleaseMat(&leftimgC3);
		cvReleaseMat(&leftimgC3a);
		lastDispImg = cvCloneMat(&dispImg);
		lastleftimg = leftimg;
		cvReleaseImage(&rightimg);
		cvReleaseMat(&rightimgC3);
		cvReleaseMat(&eig_image);
		cvReleaseMat(&temp_image);
		cvReleaseMat(&harrisCorners);
	}

	cvSave("Output/indoor1/rods.xml",   &rods,   "rods",   "rodrigues of the cam, w.r.t. start frame");
	cvSave("Output/indoor1/shifts.xml", &shifts, "shifts", "shifts of the cam, w.r.t. start frame");

	if (lastleftimg)   cvReleaseImage(&lastleftimg);
	if (lastDispImg)   cvReleaseMat(&lastDispImg);
	return status;
}

bool CvTest3DPoseEstimate::testPointClouds(){
    bool status = true;
	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped_2000_adjusted.xml");

#if 0
	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped_2000.xml");
	int numPoints = points0->rows;
	// adjusting the data set
	// turn it from meters to mm
	cvScale(points0, points0, 1000.0);
#if 0
	double _v[3] = {0, 100., 100.};
	CvMat v = cvMat(1, 3, CV_64F, _v);
	double _Temp[3*points0->rows];
	CvMat Temp = cvMat(points0->rows, 3, CV_64F, _Temp);
	cvRepeat(&v, &Temp);
	cvAdd(points0, &Temp, points0);
#endif
	// a rotation matrix to turn from x forward, z up to
	// z forward and y down
	double _X2Z[9] = {
			0, -1,  0,
			0,  0, -1,
			1,  0,  0
	};
	CvMat X2Z = cvMat(3, 3, CV_64F, _X2Z);
	CvMat *points00 = cvCreateMat(numPoints, 3, CV_64FC1);
	cvGEMM(points0, &X2Z, 1.0, NULL, 0, points00, CV_GEMM_B_T);
	cvReleaseMat(&points0);
	points0 = points00;
//	cvMatMul(&X2Z, points0, points00);
	cvSave("Data/obj1_cropped_2000_adjusted.xml", points0, "adjusted", "...so that it is more like real data");
#endif
//	CvMat * points0 =  (CvMat *)cvLoad("Data/3dPointClouds0.xml");
//	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped.xml");
	int numPoints = points0->rows;


	CvMat * points1   = cvCreateMat(numPoints,  3, CV_64FC1);
	CvMat * points1d  = cvCreateMat(numPoints,  3, CV_64FC1); // to hold the disturbed version of points1
	CvMat * points1r  = cvCreateMat(numPoints,  3, CV_64FC1);

	mEulerAngle.x = CV_PI/4.;
	mEulerAngle.y = CV_PI/3.;
	mEulerAngle.z = CV_PI/6;

	mTranslation.x = 100.;
	mTranslation.y = 10.0;
	mTranslation.z = 50.0;

	CvMat *rot    = cvCreateMat(3, 3, CV_64FC1);
	CvMat *trans  = cvCreateMat(3, 1, CV_64FC1);

//	Cv3DPoseEstimateRef peCart;
	Cv3DPoseEstimate peCart;
//	Cv3DPoseEstimateDispSpaceRef peDisp;
	Cv3DPoseEstimateDisp peDisp;
	CvMat *uvds0=NULL, *uvds1=NULL;
	CvScalar mean, std;

	if (this->mTestType == Cartesian) {
		cout << "Testing in Cartesian Space"<<endl;
		cvAvgSdv(points0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;
		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 1.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peCart.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;
	} else if (this->mTestType == Disparity){
		cout << "Testing in disparity space"<<endl;
		uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
		uvds1 = cvCreateMat(numPoints, 3, CV_64FC1);
		this->projection(points0, uvds0);
		cvAvgSdv(uvds0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 10.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peDisp.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;

		peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);
	} else if (this->mTestType == CartAndDisp) {
		cout << "testing mixed of cartesian and disparity space"<<endl;
		uvds1 = cvCreateMat(numPoints, 3, CV_64FC1);
		cvAvgSdv(points0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 1.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peDisp.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;

		peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);

	} else {
		cerr << "Unknown test type: "<< this->mTestType<<endl;
	}

	double percentageOfOutliers = mOutlierPercentage;
	double numOutliers = percentageOfOutliers*numPoints;


	double numIters = 1000;
	double maxErrorAfterLevMarq = 0.0;
	int numInliers_maxErrorAfterLevMarq=0;
	double maxErrorBeforeLevMarq = 0.0;
	int numInliers_maxErrorBeforeLevMarq=0;
	int maxNumInliers = 0;
	double errAfterLevMarq_maxNumInliers=0.0;
	double maxImprovementAfterLevMarq = 0;
	int numInliers_maxImprovementAfterLevMarq;

	double maxErrorRod = 0.0;
	int numInliers_maxErrorRod = 0;
	int testCaseNum = -1;

	int numGoodIters=0;
	for (int i=0; i<numIters; i++)
	{
		cout << "Test Case Number:  "<<i<<endl;
		if (i>0) {
			// first iter use preset value
			mEulerAngle.x = CV_PI/4.0*randReal(-1., 1.);
			mEulerAngle.y = CV_PI/4.0*randReal(-1., 1.);
			mEulerAngle.z = CV_PI/4.0*randReal(-1., 1.);

			mTranslation.x = 50. + 100.*cvRandReal(&mRng);
			mTranslation.y = 50. + 100.*cvRandReal(&mRng);
			mTranslation.z = 50. + 100.*cvRandReal(&mRng);
		}
		transform(points0, points1);

		// now randomize the first percentageOfOutliers points to make them outliers
		randomize(points1, numOutliers, mOutlierScale);

		int numInLiers;
		CvMat *inliers0=NULL, *inliers1=NULL;

		CvMat *TransformBestBeforeLevMarq;
		CvMat *TransformAfterLevMarq;
		if (this->mTestType == Cartesian) {
			disturb(points1, points1d);
			int64 t = cvGetTickCount();
			numInLiers = peCart.estimate(points0, points1d, rot, trans, inliers0, inliers1);
			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;

			TransformBestBeforeLevMarq = peCart.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peCart.getFinalTransformation();
		} else if (mTestType == Disparity){
			// convert both set of points into disparity color space
			this->projection(points1, points1d);
			disturb(points1d, uvds1);

			int64 t = cvGetTickCount();
			numInLiers = peDisp.estimate(uvds0, uvds1, rot, trans, inliers0, inliers1);

			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
			TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peDisp.getFinalTransformation();
		} else if (mTestType == CartAndDisp) {
			// convert both set of points into disparity color space
			this->projection(points1, points1d);
			disturb(points1d, uvds1);

			int64 t = cvGetTickCount();
			numInLiers = peDisp.estimateMixedPointClouds(points0, uvds1, 0, NULL, rot, trans, inliers0, inliers1);

			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
			TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peDisp.getFinalTransformation();

		} else {
			cerr << "Unknown test type "<< mTestType << endl;
		}
		if (numInLiers < 6) {
			continue;
		}
		numGoodIters++;

#if 1
		cout << "Max num of InLiers: "<< numInLiers << endl;
		cout << "Reconstructed Rotation Matrix" << endl;
		CvMatUtils::printMat(rot);
		cout << "Reconstructed Translation Matrix" << endl;
		CvMatUtils::printMat(trans);

		// calculate the relative L2 norm of the diff between true transformation and predicted transformation
		// compute rodrigues from rot and mRot
		double _rod0[3], _rod1[3];
		CvMat rod0 = cvMat(3, 1, CV_64F, _rod0);
		CvMat rod1 = cvMat(3, 1, CV_64F, _rod1);
		cvRodrigues2(&mRot, &rod0);
		cvRodrigues2(rot, &rod1);

		cout << "Rodrigues:"<<endl;
		CvMatUtils::printMat(&rod0);
		CvMatUtils::printMat(&rod1);

		double errRod   = cvNorm(&rod0, &rod1, CV_L2);
		double errTransMat = cvNorm(trans, &mTrans, CV_RELATIVE_L2);

		cout << "L2 -norm of the diff of rodrigues and trans mat are: "<<errRod<<" , "<<errTransMat<<endl;

		if (maxErrorRod < errRod) {
			maxErrorRod = errRod;
			numInliers_maxErrorRod = numInLiers;
			testCaseNum = i;
		}

		CvMat P0, P1r;

		cvReshape(points0,  &P0,  3, 0);
		cvReshape(points1r, &P1r, 3, 0);

		cvPerspectiveTransform(&P0, &P1r, TransformBestBeforeLevMarq);

		// calculate the relative L2 norm of the diff between observed and predicted points;
		double errorBeforeLevMarq = cvNorm(points1, points1r, CV_RELATIVE_L2)/numPoints;
		cout << "Average of the L2-norm of the diff between observed and prediction (Before LevMarq):  "<< errorBeforeLevMarq << endl;

		if (maxErrorBeforeLevMarq < errorBeforeLevMarq) {
			maxErrorBeforeLevMarq = errorBeforeLevMarq;
			numInliers_maxErrorBeforeLevMarq = numInLiers;
		}


		this->transform(rot, points0, trans, points1r);
		//	cout << "Reconstructed tranformed points: "<<endl;
		//	CvMatUtils::printMat(points1r);

		// calculate the relative L2 norm of the diff between observed and predicted points;
		double errorAfterLevMarq = cvNorm(points1, points1r, CV_RELATIVE_L2)/numPoints;
		cout << "Average of the L2-norm of the diff between observed and prediction (After  LevMarq):  "<< errorAfterLevMarq << endl;

		if (maxErrorAfterLevMarq < errorAfterLevMarq) {
			maxErrorAfterLevMarq = errorAfterLevMarq;
			numInliers_maxErrorAfterLevMarq = numInLiers;
		}

		if (maxNumInliers < numInLiers){
			maxNumInliers = numInLiers;
			errAfterLevMarq_maxNumInliers = errorAfterLevMarq;
		}

		if (maxImprovementAfterLevMarq < errorBeforeLevMarq - errorAfterLevMarq){
			maxImprovementAfterLevMarq = errorBeforeLevMarq - errorAfterLevMarq;
			numInliers_maxImprovementAfterLevMarq = numInLiers;
		}

		if (inliers0){
			CvMat *inliers1r = cvCreateMat(numInLiers, 3, CV_64FC1);

			CvMat P0, P1r;

			cvReshape(inliers0, &P0, 3, 0);
			cvReshape(inliers1r, &P1r, 3, 0);

			cvPerspectiveTransform(&P0, &P1r, TransformBestBeforeLevMarq);
			double errorInliersBefore = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (Before LevMarq):  "<< errorInliersBefore << endl;
#if 0
			if (maxErrorBeforeLevMarq < errorInliersBefore) {
				maxErrorBeforeLevMarq = errorInliersBefore;
				numInliers_maxErrorBeforeLevMarq = numInLiers;
			}
#endif

			cvPerspectiveTransform(&P0, &P1r, TransformAfterLevMarq);
//			this->transform(rot, inliers0, trans, inliers1r);
			//	cout << "Reconstructed tranformed points of inliers: "<<endl;
			// calculate the relative L2 norm of the diff between observed and predicted points;
			double errorInliersAfter = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (After  LevMarq):  "<< errorInliersAfter << endl;
#if 0
			if (maxErrorAfterLevMarq < errorInliersAfter) {
				maxErrorAfterLevMarq = errorInliersAfter;
				numInliers_maxErrorAfterLevMarq = numInLiers;
			}

			if (maxNumInliers < numInLiers){
				maxNumInliers = numInLiers;
				errAfterLevMarq_maxNumInliers = errorInliersAfter;
			}

			if (maxImprovementAfterLevMarq < errorInliersBefore - errorInliersAfter){
				maxImprovementAfterLevMarq = errorInliersBefore - errorInliersAfter;
				numInliers_maxImprovementAfterLevMarq = numInLiers;
			}
#endif

			cvReleaseMat(&inliers1r);
		}

#endif
	}
	cout << "max error of rodrigues: "<<maxErrorRod <<endl;
	cout << "num of inliers for the test case: "<< numInliers_maxErrorRod<<endl;
	cout << "test Case number: "<< testCaseNum <<endl;
	cout << "max error before levmarq: "<< maxErrorBeforeLevMarq << endl;
	cout << "num of inliers for the test case: "<< numInliers_maxErrorBeforeLevMarq << endl;

	cout << "max error after levmarq:  " << maxErrorAfterLevMarq  << endl;
	cout << "num of inliers for the test case: " << numInliers_maxErrorAfterLevMarq<< endl;

	cout << "max improvement after levmarq: " << maxImprovementAfterLevMarq << endl;
	cout << "num of inliers for the test case: "<<numInliers_maxImprovementAfterLevMarq << endl;

	cout << "max num of inliers: "<< maxNumInliers << endl;
	cout << "error after levmarq for the test case: "<< errAfterLevMarq_maxNumInliers << endl;

	CvTestTimer& timer = CvTestTimer::getTimer();

	timer.mNumIters = numGoodIters;
	timer.printStat();

	cvReleaseMat(&points1);
	cvReleaseMat(&points1d);
	cvReleaseMat(&points1r);

    return status;
}

int main(int argc, char **argv){
	CvTest3DPoseEstimate test3DPoseEstimate;

	if (argc >= 2) {
		char *option = argv[1];
		if (strcasecmp(option, "cartesian")==0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Cartesian;
		} else if (strcasecmp(option, "disparity")==0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Disparity;
		} else if (strcasecmp(option, "cartanddisp") == 0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::CartAndDisp;
		} else if (strcasecmp(option, "video") == 0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Video;
		} else {
			cerr << "Unknown option: "<<option<<endl;
			exit(1);
		}
	} else {
		test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Cartesian;
	}

	cout << "Testing wg3DPoseEstimate ..."<<endl;


    test3DPoseEstimate.test();

	cout << "Done testing wg3DPoseEstimate ..."<<endl;
	return 0;
}
