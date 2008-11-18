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
#include <ost_stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

// WG Ext of OpenCV
#include <CvPoseEstErrMeasDisp.h>
#include <Cv3DPoseEstimateStereo.h>
#include "PoseEstimateDisp.h"
#include "PoseEstimate.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
#include <PathRecon.h>
#include <VOSparseBundleAdj.h>

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

typedef struct {
  PyObject_HEAD
  PathRecon *pr;
  PoseEstFrameEntry*& currFrame;
} path_recon_t;

static void
path_recon_dealloc(PyObject *self)
{
    PyObject_Del(self);
}

/* Method table */
static PyMethodDef path_recon_methods[] = {
  {NULL, NULL},
};

static PyObject *
path_recon_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(path_recon_methods, self, attrname);
}

static PyTypeObject path_recon_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "path_recon",
    sizeof(path_recon_t),
    0,
    (destructor)path_recon_dealloc,
    0,
    (getattrfunc)path_recon_GetAttr,
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

PyObject *mkPathRecon(PyObject *self, PyObject *args)
{
  path_recon_t *object = PyObject_NEW(path_recon_t, &path_recon_Type);
  CvSize imgSize = cvSize(640, 480);
  object->pr = new PathRecon(imgSize);

  // mFx, mFy, mTx, mClx, mCrx, mCy);
  object->pr->mPoseEstimator.setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  //object->pr->mOutputDir = string("test/Output/indoor1");

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

  int numFrames = input.size();
  CvSize imgSize = cvSize(640, 480);
  PathRecon pathRecon(imgSize);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm

  // mFx, mFy, mTx, mClx, mCrx, mCy);
  pathRecon.mPoseEstimator.setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
//  pathRecon.mOutputDir = string("test/Output/indoor1");

  while (!input.empty()) {

    // currFrame is a reference to pathRecon.mFrameSeq.mCurrentFrame;
    PoseEstFrameEntry*& currFrame = pathRecon.mFrameSeq.mCurrentFrame;
    bool stop = false;

    if (pathRecon.mFrameSeq.mNextFrame.get()) {
      // do not need to load new images nor compute the key points again
      pathRecon.mFrameSeq.mCurrentFrame = pathRecon.mFrameSeq.mNextFrame.release();
    } else {
      // process the next stereo pair of images.
      StereoFrame stereoFrame = input.front();
      if (stereoFrame.mFrameIndex == -1) {
        // done
        stop = true;
      } else {
        currFrame = new PoseEstFrameEntry(stereoFrame.mFrameIndex);
        currFrame->mImage = stereoFrame.mImage;
        currFrame->mRightImage = stereoFrame.mRightImage;
        currFrame->mDispMap = stereoFrame.mDispMap;

        // counting how many frames we have processed
        pathRecon.mFrameSeq.mNumFrames++;

        pathRecon.goodFeaturesToTrack(*currFrame->mImage, currFrame->mDispMap,
            currFrame->mKeypoints);

        // prepare the keypoint descriptors
        pathRecon.mPoseEstimator.constructKeypointDescriptors(*currFrame->mImage, *currFrame->mKeypoints);
      }
      input.pop();
    }

    KeyFramingDecision kfd = KeyFrameBackTrack;

    if (stop == false) {
      if (pathRecon.mFrameSeq.mNumFrames==1) {
        // First frame ever, do not need to do anything more.
        pathRecon.mFrameSeq.mStartFrameIndex = currFrame->mFrameIndex;
        kfd = KeyFrameUse;
      } else {
        //
        // match the good feature points between this iteration and last key frame
        //
        vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;
        if (currFrame->mTrackableIndexPairs == NULL) {
          currFrame->mTrackableIndexPairs = new vector<pair<int, int> >();
        } else {
          currFrame->mTrackableIndexPairs->clear();
        }
        pathRecon.matchKeypoints(&trackablePairs, currFrame->mTrackableIndexPairs);
        assert(currFrame->mTrackableIndexPairs->size() == trackablePairs.size());

        cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() << endl;
        for (size_t i = 0; i < trackablePairs.size(); i++) {
          printf("(%f,%f,%f), (%f,%f,%f)\n",
            trackablePairs[i].first.x,
            trackablePairs[i].first.y,
            trackablePairs[i].first.z,
            trackablePairs[i].second.x,
            trackablePairs[i].second.y,
            trackablePairs[i].second.z);
        }

        if (currFrame->mNumTrackablePairs< defMinNumTrackablePairs) {
          cout << "Too few trackable pairs - backtracking" <<endl;
          kfd = KeyFrameBackTrack;
        } else {
          //  pose estimation given the feature point pairs
          //  note we do not do Levenberg-Marquardt here, as we are not sure if
          //  this is key frame yet.
          currFrame->mNumInliers =
            pathRecon.mPoseEstimator.estimate(*currFrame->mKeypoints, *pathRecon.getLastKeyFrame()->mKeypoints,
                *currFrame->mTrackableIndexPairs,
                currFrame->mRot, currFrame->mShift, false);

          pathRecon.mStat.mHistoInliers.push_back(currFrame->mNumInliers);

          currFrame->mInliers0 = NULL;
          currFrame->mInliers1 = NULL;
          pathRecon.fetchInliers(currFrame->mInliers1, currFrame->mInliers0);
          currFrame->mInlierIndices = pathRecon.fetchInliers();
          currFrame->mLastKeyFrameIndex = pathRecon.getLastKeyFrame()->mFrameIndex;
          cout << "num of inliers: "<< currFrame->mNumInliers <<endl;
          assert(pathRecon.getLastKeyFrame());

          // Decide if we need select a key frame by now
          kfd =
            pathRecon.keyFrameEval(currFrame->mFrameIndex, currFrame->mKeypoints->size(),
                currFrame->mNumInliers,
                currFrame->mRot, currFrame->mShift);
        }
      } // not first frame;
    } // stop == false

    bool newKeyFrame = pathRecon.keyFrameAction(kfd, pathRecon.mFrameSeq);
    if (newKeyFrame == true) {
      // only keep the last frame in the queue
      pathRecon.reduceWindowSize(1);
    }
  }

  // vector<FramePose>* framePoses = pathRecon.getFramePoses();
  // saveFramePoses(string("test/Output/indoor1/"), *framePoses);

  PyObject *r;
  r = PyList_New(pathRecon.mFramePoses.size());

  for (size_t i = 0; i < pathRecon.mFramePoses.size(); i++) {
    FramePose *fp = pathRecon.mFramePoses[i];
    PyObject *tup = PyTuple_New(3);
    PyTuple_SetItem(tup, 0, PyInt_FromLong(fp->mIndex));

    PyObject *rod = PyTuple_New(3);
    PyTuple_SetItem(rod, 0, PyFloat_FromDouble(fp->mRod.x));
    PyTuple_SetItem(rod, 1, PyFloat_FromDouble(fp->mRod.y));
    PyTuple_SetItem(rod, 2, PyFloat_FromDouble(fp->mRod.z));
    PyTuple_SetItem(tup, 1, rod);

    PyObject *shift = PyTuple_New(3);
    PyTuple_SetItem(shift, 0, PyFloat_FromDouble(fp->mShift.x));
    PyTuple_SetItem(shift, 1, PyFloat_FromDouble(fp->mShift.y));
    PyTuple_SetItem(shift, 2, PyFloat_FromDouble(fp->mShift.z));
    PyTuple_SetItem(tup, 2, shift);

    PyList_SetItem(r, i, tup);
  }

  pathRecon.printStat();

  CvTestTimer::getTimer().mNumIters = numFrames;
  CvTestTimer::getTimer().printStat();

  return r;
}

/************************************************************************/


PyObject *do_ost_do_prefilter_norm(PyObject *self, PyObject *args)
{
  const uint8_t *im = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 0));
  uint8_t *ftim = (uint8_t *)PyString_AsString(PyTuple_GetItem(args, 1));
  int xim = PyInt_AsLong(PyTuple_GetItem(args, 2));
  int yim = PyInt_AsLong(PyTuple_GetItem(args, 3));
  int ftzero = PyInt_AsLong(PyTuple_GetItem(args, 4));
  uint8_t *buf = (uint8_t *)PyString_AsString(PyTuple_GetItem(args, 5));

  ost_do_prefilter_fast_u(im, ftim, xim, yim, ftzero, buf);
  Py_RETURN_NONE;
}

PyObject *do_ost_do_stereo_sparse(PyObject *self, PyObject *args)
{
  char *refpat, *rim;
  int refpat_len, rim_len;
  int x, y, xim, yim, ftzero, dlen, tfilter_thresh, ufilter_thresh;

  PyArg_ParseTuple(args, "s#s#iiiiiiii",
                   &refpat, &refpat_len, &rim, &rim_len,
                   &x, &y, &xim, &yim, &ftzero, &dlen, &tfilter_thresh, &ufilter_thresh);
  return PyInt_FromLong(ost_do_stereo_sparse((uint8_t*)refpat, (uint8_t*)rim,
                       x, y, xim, yim, ftzero, dlen, tfilter_thresh, ufilter_thresh));
}

PyObject *grab_16x16(PyObject *self, PyObject *args)
{
  const uint8_t *im = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 0));
  int xim = PyInt_AsLong(PyTuple_GetItem(args, 1));
  int x = PyInt_AsLong(PyTuple_GetItem(args, 2));
  int y = PyInt_AsLong(PyTuple_GetItem(args, 3));

  char sub[256];
  if (1) {
    for (size_t i = 0; i < 16; i++)
      memcpy(sub + 16 * i, im + x + (y + i) * xim, 16);
  } else {
#define COPY16(N) \
    _mm_storeu_ps((float*)(sub + 16 * N), _mm_loadu_ps((float*)(im + x + (y + N) * xim)));
    COPY16(0)
    COPY16(1)
    COPY16(2)
    COPY16(3)
    COPY16(4)
    COPY16(5)
    COPY16(6)
    COPY16(7)
    COPY16(8)
    COPY16(9)
    COPY16(10)
    COPY16(11)
    COPY16(12)
    COPY16(13)
    COPY16(14)
    COPY16(15)
  }

  return PyString_FromStringAndSize(sub, 256);
}

PyObject *sad(PyObject *self, PyObject *args)
{
  const uint8_t *im0 = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 0));
  const uint8_t *im1 = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 1));

  int sad = 0;
  for (size_t i = 0; i < 256; i++)
    sad += std::abs(im0[i] - im1[i]);

  return PyFloat_FromDouble((double)sad);
}

PyObject *sad_search(PyObject *self, PyObject *args)
{
  const uint8_t *reference = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 0));
  PyObject *library = PyTuple_GetItem(args, 1);
  char *hits;
  Py_ssize_t nhits;
  PyString_AsStringAndSize(PyTuple_GetItem(args, 2), &hits, &nhits);

  int best_ci = -1;
  int best_sad = 999999;

  for (int ci = 0; ci < (int)nhits; ci++) {
    if (hits[ci]) {
      const uint8_t *im1 = (const uint8_t *)PyString_AsString(PyList_GetItem(library, ci));

      int sad;
      if (0) {
        sad = 0;
        for (size_t j = 0; j < 256; j++)
          sad += std::abs(reference[j] - im1[j]);
      } else {
        __m128i a, b, diff, total;
        total = (__m128i)_mm_setzero_ps();
        unsigned int sums[4];

#define ACCUM_SAD(offset) \
        a = (__m128i)_mm_loadu_ps((float*)(reference + (16 * offset))); \
        b = (__m128i)_mm_loadu_ps((float*)(im1 + (16 * offset))); \
        diff = _mm_sad_epu8(a, b); \
        total = _mm_add_epi32(diff, total);

        ACCUM_SAD(0)
        ACCUM_SAD(1)
        ACCUM_SAD(2)
        ACCUM_SAD(3)
        ACCUM_SAD(4)
        ACCUM_SAD(5)
        ACCUM_SAD(6)
        ACCUM_SAD(7)
        ACCUM_SAD(8)
        ACCUM_SAD(9)
        ACCUM_SAD(10)
        ACCUM_SAD(11)
        ACCUM_SAD(12)
        ACCUM_SAD(13)
        ACCUM_SAD(14)
        ACCUM_SAD(15)
        _mm_storeu_ps((float*)sums, (__m128)total);

        sad = sums[0] + sums[2];
      }

      if (sad < best_sad) {
        best_sad = sad;
        best_ci = ci;
      }
    }
  }

  if (best_ci == -1) {
    Py_RETURN_NONE;
  } else {
    return PyInt_FromLong(best_ci);
  }
}

//
// Dense stereo pair
//

PyObject *dense_stereo(PyObject *self, PyObject *args)
{
  const uint8_t *lim = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 0));
  const uint8_t *rim = (const uint8_t *)PyString_AsString(PyTuple_GetItem(args, 1));
  int w = PyInt_AsLong(PyTuple_GetItem(args, 2));
  int h = PyInt_AsLong(PyTuple_GetItem(args, 3));
  int16_t *disp = (int16_t *)PyString_AsString(PyTuple_GetItem(args, 4));

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

  free(mBufStereoPairs);
  free(mFeatureImgBufLeft);
  free(mFeatureImgBufRight);
  Py_RETURN_NONE;
}

PyObject *harris(PyObject *self, PyObject *args)
{
  char *imgdata;
  int imgdata_size, x, y;
  int corner_count;
  double quality_level, min_distance;

  if (!PyArg_ParseTuple(args, "s#iiidd", &imgdata, &imgdata_size, &x, &y, &corner_count, &quality_level, &min_distance))
    return NULL;

  IplImage* input = cvCreateImageHeader(cvSize(x, y), IPL_DEPTH_8U, 1);
  cvSetData(input, imgdata, x);
  IplImage* eig_image = cvCreateImage(cvSize(x, y), IPL_DEPTH_32F, 1);
  IplImage* temp_image = cvCreateImage(cvSize(x, y), IPL_DEPTH_32F, 1);
  CvPoint2D32f corners[corner_count];
  cvGoodFeaturesToTrack(
    input,
    eig_image,
    temp_image,
    corners,
    &corner_count,
    quality_level,
    min_distance,
    NULL,
    3,
    1);

  PyObject *r = PyList_New(corner_count);
  for (int i = 0; i < corner_count; i++) {
    PyList_SetItem(r, i, Py_BuildValue("dd",
        corners[i].x,
        corners[i].y));
  }

  cvReleaseImageHeader(&input);
  cvReleaseImage(&eig_image);
  cvReleaseImage(&temp_image);
  return r;
}

/************************************************************************/

//
// Pose Estimator
//

typedef struct {
  PyObject_HEAD
  PoseEstimateStereo *pe;
} pose_estimator_t;

static void
pose_estimator_dealloc(PyObject *self)
{
  PoseEstimateStereo *pe = ((pose_estimator_t*)self)->pe;
  delete pe;
  PyObject_Del(self);
}

PyObject *estimate(PyObject *self, PyObject *args)
{
  PoseEstimateStereo *pe = ((pose_estimator_t*)self)->pe;
  vector <Keypoint> kpa;
  vector <Keypoint> kpb;
  vector <pair<int, int> > pairs;
  int polishing = 1;

  PyObject *p_kpa, *p_kpb, *p_pairs;
  if (!PyArg_ParseTuple(args, "OOO|i", &p_kpa, &p_kpb, &p_pairs, &polishing)) return NULL;

  double x, y, z;
  for (int i = 0; i < PyList_Size(p_kpa); i++) {
    PyArg_ParseTuple(PyList_GetItem(p_kpa, i), "ddd", &x, &y, &z);
    kpa.push_back(Keypoint(x, y, z, 0., 0., NULL));
  }
  for (int i = 0; i < PyList_Size(p_kpb); i++) {
    PyArg_ParseTuple(PyList_GetItem(p_kpb, i), "ddd", &x, &y, &z);
    kpb.push_back(Keypoint(x, y, z, 0., 0., NULL));
  }
  for (int i = 0; i < PyList_Size(p_pairs); i++) {
    int a, b;
    PyArg_ParseTuple(PyList_GetItem(p_pairs, i), "ii", &a, &b);
    pairs.push_back(make_pair(a, b));
  }

  double _mRot[9];
  double _mShift[3];
  CvMat rot   = cvMat(3, 3, CV_64FC1, _mRot);
  CvMat shift = cvMat(3, 1, CV_64FC1, _mShift);

  int inliers = pe->estimate(kpa, kpb, pairs, rot, shift, polishing);

  return Py_BuildValue("(i,(ddddddddd),(ddd))",
                       inliers,
                       _mRot[0], _mRot[1], _mRot[2],_mRot[3], _mRot[4], _mRot[5],_mRot[6], _mRot[7], _mRot[8],
                       _mShift[0], _mShift[1], _mShift[2]);
}

PyObject *inliers(PyObject *self, PyObject *args)
{
  PoseEstimateStereo *pe = ((pose_estimator_t*)self)->pe;

  CvMat *inliers0, *inliers1;
  pe->getInliers(inliers0, inliers1);

  PyObject *r;
  if (inliers0 != NULL && inliers1 != NULL) {
    r = PyList_New(inliers0->rows);
    for (int i = 0; i < inliers0->rows; i++) {
      PyList_SetItem(r, i, Py_BuildValue("((ddd),(ddd))",
          cvGetReal2D(inliers0, i, 0),
          cvGetReal2D(inliers0, i, 1),
          cvGetReal2D(inliers0, i, 2),
          cvGetReal2D(inliers1, i, 0),
          cvGetReal2D(inliers1, i, 1),
          cvGetReal2D(inliers1, i, 2)
          ));
    }
  } else {
    r = PyList_New(0);
  }

  return r;
}

/* Method table */
static PyMethodDef pose_estimator_methods[] = {
  {"estimate", estimate, METH_VARARGS},
  {"inliers", inliers, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
pose_estimator_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(pose_estimator_methods, self, attrname);
}

static PyTypeObject pose_estimator_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "pose_estimator",
    sizeof(pose_estimator_t),
    0,
    (destructor)pose_estimator_dealloc,
    0,
    (getattrfunc)pose_estimator_GetAttr,
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

PyObject *pose_estimator(PyObject *self, PyObject *args)
{
  pose_estimator_t *object = PyObject_NEW(pose_estimator_t, &pose_estimator_Type);
  object->pe = new PoseEstimateStereo();

  double Fx, Fy, Tx, Clx, Crx, Cy;
  if (!PyArg_ParseTuple(args, "dddddd", &Fx, &Fy, &Tx, &Clx, &Crx, &Cy)) return NULL;
  object->pe->setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
  object->pe->setInlierErrorThreshold(3.0);

  return (PyObject*)object;
}
/************************************************************************/
#include "imwin.h"

typedef struct {
  PyObject_HEAD
  imWindow *iw;
} imWindow_t;

PyObject *iwDisplayImage(PyObject *self, PyObject *args)
{
  imWindow *iw = ((imWindow_t*)self)->iw;
  int imdata_size;
  unsigned char *imdata;
  PyObject *pts;
  if (!PyArg_ParseTuple(args, "s#O", &imdata, &imdata_size, &pts)) return NULL;
  iw->DisplayImage(imdata, 640, 480, 0, MONOCHROME);

  // fl_color(FL_RED);
  // fl_line(10, 10, 100, 100);
  //
  int c, x, y;
  for (int i = 0; i < PyList_Size(pts); i++) {
    PyArg_ParseTuple(PyList_GetItem(pts, i), "iii", &c, &x, &y);
    if (c == 0)
      fl_color(FL_RED);
    else
      fl_color(FL_GREEN);
    fl_line(x-4, y, x+4, y);
    fl_line(x, y-4, x, y+4);
  }

  fltk_check();
  Py_RETURN_NONE;
}

static void
imWindow_dealloc(PyObject *self)
{
  PyObject_Del(self);
}

/* Method table */
static PyMethodDef imWindow_methods[] = {
  {"DisplayImage", iwDisplayImage, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
imWindow_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(imWindow_methods, self, attrname);
}

static PyTypeObject imWindow_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "imWindow",
    sizeof(imWindow_t),
    0,
    (destructor)imWindow_dealloc,
    0,
    (getattrfunc)imWindow_GetAttr,
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

PyObject *mkimWindow(PyObject *self, PyObject *args)
{
  imWindow_t *object = PyObject_NEW(imWindow_t, &imWindow_Type);
  object->iw = new imWindow(640, 480);
  object->iw->show();

  return (PyObject*)object;
}


/************************************************************************/

static PyMethodDef methods[] = {
  {"visual_odometry", visual_odometry, METH_VARARGS},
  {"StereoFrame", stereo_frame, METH_VARARGS},
  {"PathRecon", mkPathRecon, METH_VARARGS},
  {"ost_do_prefilter_norm", do_ost_do_prefilter_norm, METH_VARARGS},
  {"ost_do_stereo_sparse", do_ost_do_stereo_sparse, METH_VARARGS},
  {"grab_16x16", grab_16x16, METH_VARARGS},
  {"sad", sad, METH_VARARGS},
  {"sad_search", sad_search, METH_VARARGS},
  {"dense_stereo", dense_stereo, METH_VARARGS},
  {"harris", harris, METH_VARARGS},
  {"imWindow", mkimWindow, METH_VARARGS},

  {"pose_estimator", pose_estimator, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void initvotools()
{
    PyObject *m, *d;

    m = Py_InitModule("votools", methods);
    d = PyModule_GetDict(m);
}
