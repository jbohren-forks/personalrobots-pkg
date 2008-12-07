#include <Python.h>

#include <iostream>
#include <vector>
#include <queue>

#include <boost/foreach.hpp>

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

using namespace cv;
using namespace std;

#define JDC_DEBUG 1

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
// FramePose
//

typedef struct {
  PyObject_HEAD
  FramePose *fp;
} frame_pose_t;

static void
frame_pose_dealloc(PyObject *self)
{
  FramePose *fp = ((frame_pose_t*)self)->fp;
  delete fp;
  PyObject_Del(self);
}

/* Method table */
static PyMethodDef frame_pose_methods[] = {
  {NULL, NULL},
};

static PyObject *
frame_pose_GetAttr(PyObject *self, char *attrname)
{
    if (strcmp(attrname, "M") == 0) {
      FramePose *fp = ((frame_pose_t*)self)->fp;
      PyObject *r = PyList_New(16);
      for (int i = 0; i < 16; i++)
        PyList_SetItem(r, i, PyFloat_FromDouble(fp->transf_local_to_global_data_[i]));
      return r;
    } else {
      return Py_FindMethod(frame_pose_methods, self, attrname);
    }
}

static PyTypeObject frame_pose_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "frame_pose",
    sizeof(frame_pose_t),
    0,
    (destructor)frame_pose_dealloc,
    0,
    (getattrfunc)frame_pose_GetAttr,
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

PyObject *frame_pose(PyObject *self, PyObject *args)
{
  frame_pose_t *object = PyObject_NEW(frame_pose_t, &frame_pose_Type);

  int i;
  double M[16];
  if (!PyArg_ParseTuple(args, "i(dddddddddddddddd)", &i,
        &M[0],
        &M[1],
        &M[2],
        &M[3],
        &M[4],
        &M[5],
        &M[6],
        &M[7],
        &M[8],
        &M[9],
        &M[10],
        &M[11],
        &M[12],
        &M[13],
        &M[14],
        &M[15])) return NULL;
  CvMat m = cvMat(4, 4, CV_64FC1, M);
  object->fp = new FramePose(i);
  cvCopy(&m, &object->fp->transf_local_to_global_);

  return (PyObject*)object;
}


/************************************************************************/

//
// PointTrack
//

typedef struct {
  PyObject_HEAD
  PointTrack *pt;
} point_track_t;

static void
point_track_dealloc(PyObject *self)
{
  PointTrack *pt = ((point_track_t*)self)->pt;
  delete pt;
  PyObject_Del(self);
}

PyObject *extend(PyObject *self, PyObject *args)
{
  PointTrack *pt = ((point_track_t*)self)->pt;
  int f0;
  CvPoint3D64f o0;
  if (!PyArg_ParseTuple(args, "i(ddd)", &f0, &o0.x, &o0.y, &o0.z)) return NULL;
  PointTrackObserv *pto0 = new PointTrackObserv(f0, o0, 0);
  pt->extend(pto0);
  Py_RETURN_NONE;
}

/* Method table */
static PyMethodDef point_track_methods[] = {
  { "extend", extend, METH_VARARGS },
  { NULL, NULL },
};

static PyObject *
point_track_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(point_track_methods, self, attrname);
}

static PyTypeObject point_track_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "point_track",
    sizeof(point_track_t),
    0,
    (destructor)point_track_dealloc,
    0,
    (getattrfunc)point_track_GetAttr,
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

PyObject *point_track(PyObject *self, PyObject *args)
{
  point_track_t *object = PyObject_NEW(point_track_t, &point_track_Type);

  int f0, f1, trackid;
  CvPoint3D64f o0, o1, p;
  if (!PyArg_ParseTuple(args, "i(ddd)i(ddd)(ddd)i", &f0, &o0.x, &o0.y, &o0.z, &f1, &o1.x, &o1.y, &o1.z, &p.x, &p.y, &p.z, &trackid)) return NULL;
  PointTrackObserv *pto0 = new PointTrackObserv(f0, o0, 0);
  PointTrackObserv *pto1 = new PointTrackObserv(f1, o1, 0);
  object->pt = new PointTrack(pto0, pto1, p, trackid);

  return (PyObject*)object;
}


/************************************************************************/

//
// Pose Estimator
//

typedef struct {
  PyObject_HEAD
  PoseEstimateStereo *pe;
#if JDC_DEBUG==1 // jdc
  LevMarqSparseBundleAdj *sba_;
  SBAVisualizer          *vis_;
#endif
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

  CvMat *inliers0 = NULL, *inliers1 = NULL;
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

PyObject *setInlierErrorThreshold(PyObject *self, PyObject *args)
{
  PoseEstimateStereo *pe = ((pose_estimator_t*)self)->pe;
  double thresh;
  if (!PyArg_ParseTuple(args, "d", &thresh))
    return NULL;
  pe->setInlierErrorThreshold(thresh);
  Py_RETURN_NONE;
}

static vector<FramePose*> fpl_p2c(PyObject *o)
{
  vector<FramePose*> r;
  for (Py_ssize_t i = 0; i < PyList_Size(o); i++) {
    PyObject *oi = PyList_GetItem(o, i);
    r.push_back(((frame_pose_t*)oi)->fp);
  }
  return r;
}

PyObject *sba(PyObject *self, PyObject *args)
{
#if JDC_DEBUG==1  // commented out by jdc
#else
  PoseEstimateStereo *pe = ((pose_estimator_t*)self)->pe;
  CvMat cartToDisp;
  CvMat dispToCart;
  pe->getProjectionMatrices(&cartToDisp, &dispToCart);

  int full_free_window_size  = 1;
  int full_fixed_window_size = 1;
  int max_num_iters = 5;
  double epsilon = DBL_EPSILON;
  CvTermCriteria term_criteria = cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,max_num_iters,epsilon);
  LevMarqSparseBundleAdj sba(&dispToCart, &cartToDisp, full_free_window_size, full_fixed_window_size, term_criteria);
#endif

  PyObject *ofixed, *ofree, *otracks;
  if (!PyArg_ParseTuple(args, "OOO", &ofixed, &ofree, &otracks)) return NULL;
  vector<FramePose*> fixed_frames = fpl_p2c(ofixed);
  vector<FramePose*> free_frames = fpl_p2c(ofree);
  PointTracks tracks;
  tracks.current_frame_index_ = 0;
  tracks.oldest_frame_index_in_tracks_ = 0;
  for (Py_ssize_t i = 0; i < PyList_Size(otracks); i++) {
    PyObject *oi = PyList_GetItem(otracks, i);
    if (!PyObject_IsInstance(oi, (PyObject*)&point_track_Type)) {
      PyErr_SetString(PyExc_TypeError, "expecting list of point_track");
      return NULL;
    }
    PointTrack *pt = ((point_track_t*)oi)->pt;
    tracks.tracks_.push_back(pt);
  }

#if JDC_DEBUG==1 // jdc
  LevMarqSparseBundleAdj* sba = ((pose_estimator_t*)self)->sba_;
  sba->optimize(&free_frames, &fixed_frames, &tracks);

  SBAVisualizer* vis = ((pose_estimator_t*)self)->vis_;
  // set frame poses, point tracks and maps from index to frame poses
  vector<FramePose* > frame_poses;
  BOOST_FOREACH(FramePose *fp, free_frames) {
    frame_poses.push_back(fp);
    vis->map_index_to_FramePose_->insert(make_pair(fp->mIndex,fp));
  }
  BOOST_FOREACH(FramePose *fp, fixed_frames) {
    frame_poses.push_back(fp);
    vis->map_index_to_FramePose_->insert(make_pair(fp->mIndex, fp));;
  }
  vis->framePoses = &frame_poses;
  vis->tracks = &tracks;
  int current_frame_index = free_frames.back()->mIndex;

  // make sure the image buffers is allocated to the right sizes
  vis->canvasTracking.Allocate(640, 480);
  // clear the image
  cvSet(vis->canvasTracking.Ipl(), cvScalar(0,0,0));
  { // annotation on the canvas
    char info[256];
    CvPoint org = cvPoint(0, 475);
    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, .5, .4);
    sprintf(info, "%04d, nTrcks=%d",
        current_frame_index, tracks.tracks_.size());

    cvPutText(vis->canvasTracking.Ipl(), info, org, &font, CvMatUtils::yellow);
  }
  sprintf(vis->poseEstFilename,  "%s/poseEst-%04d.png", vis->outputDirname.c_str(),
      current_frame_index);
  vis->slideWindowFront = free_frames.front()->mIndex;
  vis->drawTrackTrajectories(current_frame_index);
  vis->show();
  vis->save();
  vis->reset();
#else
  sba.optimize(&free_frames, &fixed_frames, &tracks);
#endif

  Py_RETURN_NONE;
}

/* Method table */
static PyMethodDef pose_estimator_methods[] = {
  {"estimate", estimate, METH_VARARGS},
  {"inliers", inliers, METH_VARARGS},
  {"sba", sba, METH_VARARGS},
  {"setInlierErrorThreshold", setInlierErrorThreshold, METH_VARARGS},
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

#if JDC_DEBUG==1 // jdc
  CvMat cartToDisp;
  CvMat dispToCart;
  object->pe->getProjectionMatrices(&cartToDisp, &dispToCart);
  int full_free_window_size  = 1;
  int full_fixed_window_size = 1;
  int max_num_iters = 5;
  double epsilon = DBL_EPSILON;
  CvTermCriteria term_criteria = cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,max_num_iters,epsilon);
  object->sba_ = new LevMarqSparseBundleAdj(&dispToCart, &cartToDisp, full_free_window_size, full_fixed_window_size, term_criteria);

  object->vis_ = new SBAVisualizer((PoseEstimateDisp&)*object->pe, NULL, NULL, NULL);
  object->vis_->map_index_to_FramePose_ =
    new boost::unordered_map<int, FramePose*>();
  object->vis_->outputDirname = string("Output/james4/");
#endif

  return (PyObject*)object;
}

/************************************************************************/

#include "imwin.h"

typedef struct {
  PyObject_HEAD
  imWindow *iw;
  int mouse_x, mouse_y, mouse_b;
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

PyObject *iwMouse(PyObject *self, PyObject *args)
{
  imWindow_t *iw = ((imWindow_t*)self);
  return Py_BuildValue("iii", iw->mouse_x, iw->mouse_y, iw->mouse_b);
}

static void
imWindow_dealloc(PyObject *self)
{
  PyObject_Del(self);
}

/* Method table */
static PyMethodDef imWindow_methods[] = {
  {"DisplayImage", iwDisplayImage, METH_VARARGS},
  {"mouse", iwMouse, METH_VARARGS},
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

int my_bhandler(int e, int x, int y, int b, int mod, imWindow *imw)
{
  imWindow_t *object = (imWindow_t *)imw->data;
  object->mouse_x = x;
  object->mouse_y = y;
  object->mouse_b = b;
  return 0;
}

PyObject *mkimWindow(PyObject *self, PyObject *args)
{
  imWindow_t *object = PyObject_NEW(imWindow_t, &imWindow_Type);
  char *title = (char*)"None";
  PyArg_ParseTuple(args, "s", &title);
  object->iw = new imWindow(640, 480, title);
  object->iw->data = (void*)object;
  object->iw->ButtonHandler(my_bhandler);
  object->iw->show();

  return (PyObject*)object;
}

/************************************************************************/

static PyMethodDef methods[] = {
  {"ost_do_prefilter_norm", do_ost_do_prefilter_norm, METH_VARARGS},
  {"ost_do_stereo_sparse", do_ost_do_stereo_sparse, METH_VARARGS},
  {"grab_16x16", grab_16x16, METH_VARARGS},
  {"sad", sad, METH_VARARGS},
  {"sad_search", sad_search, METH_VARARGS},
  {"dense_stereo", dense_stereo, METH_VARARGS},
  {"harris", harris, METH_VARARGS},
  {"point_track", point_track, METH_VARARGS},
  {"frame_pose", frame_pose, METH_VARARGS},
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
