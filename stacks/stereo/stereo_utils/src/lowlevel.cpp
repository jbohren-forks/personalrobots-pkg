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

using namespace cv;
using namespace std;

/************************************************************************/

PyObject *do_ost_do_prefilter_norm(PyObject *self, PyObject *args)
{
  const uint8_t *im;
  uint8_t *ftim;
  int xim;
  int yim;
  int ftzero;
  uint8_t *buf;
  int im_size, ftim_size, buf_size;
  if (!PyArg_ParseTuple(args, "s#s#iiis#", &im, &im_size, &ftim, &ftim_size, &xim, &yim, &ftzero, &buf, &buf_size)) {
    return NULL;
  }

  assert(im_size == (xim * yim));
  assert(ftim_size == (xim * yim));

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
  const uint8_t *im;
  int im_size;
  int xim;
  int x;
  int y;
  if (!PyArg_ParseTuple(args, "s#iii", &im, &im_size, &xim, &x, &y))
    return NULL;

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
  const uint8_t *reference;
  PyObject *library;
  int reference_size;
  char *hits;
  int nhits;
  if (!PyArg_ParseTuple(args, "s#Os#", &reference, &reference_size, &library, &hits, &nhits))
    return NULL;

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

PyObject *buffer(PyObject *self, PyObject *args)
{
  int size;
  if (!PyArg_ParseTuple(args, "i", &size))
    return NULL;
  char *original = new char[size + 16];
  char *ptr = (char*)(((size_t)original + 15) & ~15);
  return Py_BuildValue("NL", PyBuffer_FromReadWriteMemory(ptr, size), (long long)original);
}

PyObject *release(PyObject *self, PyObject *args)
{
  long long ptr;
  if (!PyArg_ParseTuple(args, "L", &ptr))
    return NULL;
  delete (void*)ptr;
  Py_RETURN_NONE;
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

  {"buffer", buffer, METH_VARARGS},
  {"release", release, METH_VARARGS},

  {NULL, NULL, NULL},
};

extern "C" void init_stereo_utils_lowlevel()
{
    PyObject *m, *d;

    m = Py_InitModule("_stereo_utils_lowlevel", methods);
    d = PyModule_GetDict(m);
}
