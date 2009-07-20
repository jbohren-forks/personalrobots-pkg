#include <Python.h>

#include <stdlib.h>
#include <malloc.h>
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#ifdef WIN32
#pragma warning (disable: 4267 4244 4800 4996)
#include <time.h>
#else
#include <sys/time.h>
#endif
#include "imwin/im3Dwin.h"
#include "stereolib.h"
#include "dcam/stereodcam.h"

#include <cv.h>
#include <cxmisc.h>
#include <cvaux.h>
#include <highgui.h>

using namespace std;

struct dcam_t {
  PyObject_HEAD
  dcam::StereoDcam *dev;
};

/* dcam */

static void dcam_dealloc(PyObject *self)
{
  // dcam::fini();
  dcam_t *pc = (dcam_t*)self;
  delete pc->dev;
  PyObject_Del(self);
}

static uint8_t *getdata(ImageData *id)
{
  // left window display, try rect, then raw
  if (id->imRectColorType != COLOR_CODING_NONE)
      return id->imRectColor;
  else if (id->imRectType != COLOR_CODING_NONE)
      return id->imRect;
  else if (id->imColorType != COLOR_CODING_NONE)
      return id->imColor;
  else if (id->imType != COLOR_CODING_NONE)
      return id->im;
  else
      return NULL;
}
static size_t getdatasize(ImageData *id)
{
  // left window display, try rect, then raw
  if (id->imRectColorType != COLOR_CODING_NONE)
      return id->imRectColorSize;
  else if (id->imRectType != COLOR_CODING_NONE)
      return id->imRectSize;
  else if (id->imColorType != COLOR_CODING_NONE)
      return id->imColorSize;
  else if (id->imType != COLOR_CODING_NONE)
      return id->imSize;
  else
      return 0;
}

static PyObject *dcam_getImage(PyObject *self, PyObject *args)
{
  dcam_t *ps = (dcam_t*)self;

  bool r = ps->dev->getImage(500);
  if (!r) {
    Py_RETURN_NONE;
  } else {
      StereoData *stIm = ps->dev->stIm;

      int w = stIm->imWidth;
      int h = stIm->imHeight;

    uint8_t *Ld = NULL, *Rd = NULL;   // left data, right data

    Ld = getdata(stIm->imLeft);
    Rd = getdata(stIm->imRight);

    PyObject *left, *right, *disparity;
    if (Ld != NULL) {
      left = PyBuffer_FromMemory(Ld, getdatasize(stIm->imLeft));
    } else {
      left = Py_None;
      Py_INCREF(left);
    }
    if (Rd != NULL) {
      right = PyBuffer_FromMemory(Rd, getdatasize(stIm->imRight));
    } else {
      right = Py_None;
      Py_INCREF(right);
    }
    if (stIm->hasDisparity) {
      disparity = Py_BuildValue("Nii", PyBuffer_FromMemory(stIm->imDisp, stIm->imDispSize), stIm->dpp, stIm->numDisp);
    } else {
      disparity = Py_None;
      Py_INCREF(disparity);
    }
    return Py_BuildValue("iiNNN",
                         w,
                         h,
                         left,
                         right,
                         disparity);
  }
#if 0
  printf("%p\n", stIm->imLeft->imRaw);
  printf("%p\n", stIm->imLeft->im);
  printf("%p\n", stIm->imLeft->imColor);
  printf("%p\n", stIm->imLeft->imRect);
  printf("%p\n", stIm->imLeft->imRectColor);
#endif
}

static PyObject *dcam_stop(PyObject *self, PyObject *args)
{
  dcam_t *ps = (dcam_t*)self;
  ps->dev->stop();
  Py_RETURN_NONE;
}

static PyObject *retParameters(PyObject *self, PyObject *args)
{
  dcam_t *ps = (dcam_t*)self;
  return PyString_FromString(ps->dev->retParameters());
}

static struct PyMethodDef dcam_methods[] =
{
  {"getImage", dcam_getImage, METH_VARARGS},
  {"retParameters",     retParameters, METH_VARARGS},
  {"stop",     dcam_stop, METH_VARARGS},
  {NULL,          NULL}
};

static PyTypeObject dcam_Type = {
  PyObject_HEAD_INIT(&PyType_Type)
  0,                                     /*size*/
  "dcam.dcam",                           /*name*/
  sizeof(dcam_t),                        /*basicsize*/
};

static void dcam_specials(void)
{
  dcam_Type.tp_dealloc = dcam_dealloc;
  dcam_Type.tp_methods = dcam_methods;
  // dcam_Type.tp_getset = dcam_getseters;
}

static PyObject *make_dcam(PyObject *self, PyObject *args)
{
  dcam_t *ps = PyObject_NEW(dcam_t, &dcam_Type);
  char *str_pmode;

  if (!PyArg_ParseTuple(args, "s", &str_pmode))
    return NULL;

  dcam::init();

  // This code copied from ost.cpp - it is the initialization sequence
  // for the camera.
  //
  int devIndex = 0;
  ps->dev = new dcam::StereoDcam(dcam::getGuid(devIndex));
  ps->dev->setRangeMax(4.0);	// in meters
  ps->dev->setRangeMin(0.5);	// in meters
  videre_proc_mode_t pmode = PROC_MODE_RECTIFIED;
  if (strcmp(str_pmode, "disparity") == 0)
    pmode = PROC_MODE_DISPARITY;
  else if (strcmp(str_pmode, "disparity_raw") == 0)
    pmode = PROC_MODE_DISPARITY_RAW;
  else if (strcmp(str_pmode, "rectified") == 0)
    pmode = PROC_MODE_RECTIFIED;
  else if (strcmp(str_pmode, "none") == 0)
    pmode = PROC_MODE_NONE;
  else if (strcmp(str_pmode, "test") == 0)
    pmode = PROC_MODE_TEST;
  else {
    PyErr_SetString(PyExc_TypeError, "mode must be none/test/rectified ");
    return NULL;
  }
  dc1394video_mode_t videoMode;	// current video mode
  dc1394framerate_t videoRate;	// current video rate
  videoMode = VIDERE_STEREO_640x480;
  videoRate = DC1394_FRAMERATE_15;
  ps->dev->setFormat(videoMode, videoRate); 
  ps->dev->setMaxAutoVals(100,48);
  ps->dev->setGain(0,true);
  ps->dev->setExposure(0,true);
  ps->dev->setBrightness(0,true);
  ps->dev->setProcMode(pmode);
  ps->dev->start();

  return (PyObject*)ps;
}

static PyMethodDef methods[] = {
  {"dcam", make_dcam, METH_VARARGS},
  {NULL, NULL},
};

extern "C" void initdcam()
{
  dcam_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  dcam_specials();
  if (PyType_Ready(&dcam_Type) < 0)
    return;
  Py_InitModule("dcam", methods);
}
