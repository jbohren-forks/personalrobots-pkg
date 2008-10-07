#include <Python.h>

#include <iostream>
#include <vector>
#include <queue>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
// the google wrapper
#include <opencv/cvwimage.h>

#include <opencv/cxcore.hpp>

#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

// WG Ext of OpenCV
#include <CvPoseEstErrMeasDisp.h>
#include <Cv3DPoseEstimateStereo.h>
#include "PoseEstimateDisp.h"
#include "PoseEstimate.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
#include <PathRecon.h>
#include <VisOdomBundleAdj.h>

#include <VisOdom.h>

using namespace cv::willow;

// star detector
//#include <detector.h>

using namespace cv;
using namespace std;


bool testVideo(queue <StereoFrame> input) {
  bool status = false;

  return status;
}
/************************************************************************/
typedef struct {
  PyObject_HEAD
  StereoFrame *sf;
} stereo_frame_t;

static void
stereo_frame_dealloc(PyObject *self)
{
    PyObject_Del(self);
}

/* Method table */
static PyMethodDef stereo_frame_methods[] = {
  //{"detect", detect, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
stereo_frame_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(stereo_frame_methods, self, attrname);
}

static PyTypeObject stereo_frame_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "stereo_frame",
    sizeof(stereo_frame_t),
    0,
    (destructor)stereo_frame_dealloc,
    0,
    (getattrfunc)stereo_frame_GetAttr,
    0,
    0,
    0, // repr
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,
    
    0,
    
    Py_TPFLAGS_CHECKTYPES,

    0,
    0,
    0,
    0

    /* the rest are NULLs */
};

static bool getDisparityMap( WImageBuffer1_b& leftImage, WImageBuffer1_b& rightImage, WImageBuffer1_16s& dispMap) {
	bool status = true;

  int w = leftImage.Width();
  int h = rightImage.Height();

  if (w != rightImage.Width() || h != rightImage.Height()) {
		cerr << __PRETTY_FUNCTION__ <<"(): size of images incompatible. "<< endl;
		return false;
	}

	//
	// Try Kurt's dense stereo pair
	//
	const uint8_t *lim = leftImage.ImageData();
	const uint8_t *rim = rightImage.ImageData();

	int16_t* disp    = dispMap.ImageData();
	int16_t* textImg = NULL;

	static const int mFTZero       = 31;		//< max 31 cutoff for prefilter value
	static const int mDLen         = 64;		//< 64 disparities
	static const int mCorr         = 15;		//< correlation window size
	static const int mTextThresh   = 10;		//< texture threshold
	static const int mUniqueThresh = 15;		//< uniqueness threshold

  // scratch images
	uint8_t *mBufStereoPairs     = new uint8_t[h*mDLen*(mCorr+5)]; // local storage for the stereo pair algorithm
	uint8_t *mFeatureImgBufLeft  = new uint8_t[w*h];
	uint8_t *mFeatureImgBufRight = new uint8_t[w*h];

	// prefilter
	do_prefilter((uint8_t *)lim, mFeatureImgBufLeft, w, h, mFTZero, mBufStereoPairs);
	do_prefilter((uint8_t *)rim, mFeatureImgBufRight, w, h, mFTZero, mBufStereoPairs);

	// stereo
	do_stereo(mFeatureImgBufLeft, mFeatureImgBufRight, disp, textImg, w, h,
			mFTZero, mCorr, mCorr, mDLen, mTextThresh, mUniqueThresh, mBufStereoPairs);

	return status;
}

// Create a StereoFrame given a framenumber and a pointer to left and right images
// computes the disparity map

PyObject *stereo_frame(PyObject *self, PyObject *args)
{
  stereo_frame_t *object = PyObject_NEW(stereo_frame_t, &stereo_frame_Type);
  object->sf = new StereoFrame();

  object->sf->mFrameIndex = PyLong_AsLong(PyTuple_GetItem(args, 0));
  object->sf->mImage = new WImageBuffer1_b();
  object->sf->mImage->SetIpl(cvLoadImage(PyString_AsString(PyTuple_GetItem(args,1)), CV_LOAD_IMAGE_GRAYSCALE));
  object->sf->mRightImage = new WImageBuffer1_b();
  object->sf->mRightImage->SetIpl(cvLoadImage(PyString_AsString(PyTuple_GetItem(args,2)), CV_LOAD_IMAGE_GRAYSCALE));

  int w = object->sf->mImage->Width();
  int h = object->sf->mImage->Height();
  object->sf->mDispMap = new WImageBuffer1_16s(w, h);

  getDisparityMap(*object->sf->mImage, *object->sf->mRightImage, *object->sf->mDispMap);
  return (PyObject*)object;
}

/************************************************************************/

PyObject *visual_odometry(PyObject *self, PyObject *args)
{
  queue <StereoFrame> input;
  PyObject *pi = PyTuple_GetItem(args, 0);

  for (int i = 0; i < PyList_Size(pi); i++) {
    PyObject *o = PyList_GetItem(pi,i);
    StereoFrame s = *(((stereo_frame_t *)o)->sf);
    input.push(s);
  }

  CvSize imgSize = cvSize(640, 480);
  PathRecon pathRecon(imgSize);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm

  // mFx, mFy, mTx, mClx, mCrx, mCy);
  pathRecon.mPoseEstimator.setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  pathRecon.mOutputDir = string("test/Output/indoor1");

  pathRecon.track(input);

  vector<FramePose>* framePoses = pathRecon.getFramePoses();

  // saveFramePoses(string("test/Output/indoor1/"), *framePoses);

  PyObject *r;
  r = PyList_New(pathRecon.mFramePoses.size());

  for (int i = 0; i < pathRecon.mFramePoses.size(); i++) {
    FramePose &fp = pathRecon.mFramePoses[i];
    PyObject *tup = PyTuple_New(3);
    PyTuple_SetItem(tup, 0, PyInt_FromLong(fp.mIndex));

    PyObject *rod = PyTuple_New(3);
    PyTuple_SetItem(rod, 0, PyFloat_FromDouble(fp.mRod.x));
    PyTuple_SetItem(rod, 1, PyFloat_FromDouble(fp.mRod.y));
    PyTuple_SetItem(rod, 2, PyFloat_FromDouble(fp.mRod.z));
    PyTuple_SetItem(tup, 1, rod);

    PyObject *shift = PyTuple_New(3);
    PyTuple_SetItem(shift, 0, PyFloat_FromDouble(fp.mShift.x));
    PyTuple_SetItem(shift, 1, PyFloat_FromDouble(fp.mShift.y));
    PyTuple_SetItem(shift, 2, PyFloat_FromDouble(fp.mShift.z));
    PyTuple_SetItem(tup, 2, shift);

    PyList_SetItem(r, i, tup);
  }

  pathRecon.printStat();

  return r;
}

/************************************************************************/

static PyMethodDef methods[] = {
  {"visual_odometry", visual_odometry, METH_VARARGS},
  {"StereoFrame", stereo_frame, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void initvisual_odometry()
{
    PyObject *m, *d;

    m = Py_InitModule("visual_odometry", methods);
    d = PyModule_GetDict(m);
}
