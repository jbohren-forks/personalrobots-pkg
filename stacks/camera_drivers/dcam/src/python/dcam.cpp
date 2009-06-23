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
  dcam_t *pc = (dcam_t*)self;
  delete pc->dev;
  PyObject_Del(self);
}

#if 0
static PyObject *dcam_repr(PyObject *self)
{
  dcam_t *cva = (dcam_t*)self;
  IplImage* ipl = (IplImage*)(cva->a);
  char str[1000];
  sprintf(str, "<dcam(");
  char *d = str + strlen(str);
  sprintf(d, "nChannels=%d ", ipl->nChannels);
  d += strlen(d);
  sprintf(d, "width=%d ", ipl->width);
  d += strlen(d);
  sprintf(d, "height=%d ", ipl->height);
  d += strlen(d);
  sprintf(d, "widthStep=%d ", ipl->widthStep);
  d += strlen(d);
  sprintf(d, ")>");
  return PyString_FromString(str);
}

static PyObject *dcam_tostring(PyObject *self, PyObject *args)
{
  IplImage *i;
  if (!convert_to_IplImage(self, &i, "self"))
    return NULL;
  if (i == NULL)
    return NULL;
  int bps;
  switch (i->depth) {
  case IPL_DEPTH_8U:
  case IPL_DEPTH_8S:
    bps = 1;
    break;
  case IPL_DEPTH_16U:
  case IPL_DEPTH_16S:
    bps = 2;
    break;
  case IPL_DEPTH_32S:
  case IPL_DEPTH_32F:
    bps = 4;
    break;
  case IPL_DEPTH_64F:
    bps = 8;
    break;
  default:
    bps = 0;
    assert(0);
  }
  int bpl = i->width * i->nChannels * bps;
  int l = bpl * i->height;
  char *s = new char[l];
  int y;
  for (y = 0; y < i->height; y++) {
    memcpy(s + y * bpl, i->imageData + y * i->widthStep, bpl);
  }
  return PyString_FromStringAndSize(s, l);
}

static PyObject *dcam_getnChannels(dcam_t *cva)
{
  return PyInt_FromLong(((IplImage*)(cva->a))->nChannels);
}
static PyObject *dcam_getwidth(dcam_t *cva)
{
  return PyInt_FromLong(((IplImage*)(cva->a))->width);
}
static PyObject *dcam_getheight(dcam_t *cva)
{
  return PyInt_FromLong(((IplImage*)(cva->a))->height);
}
static PyObject *dcam_getdepth(dcam_t *cva)
{
  return PyInt_FromLong(((IplImage*)(cva->a))->depth);
}
static PyObject *dcam_getorigin(dcam_t *cva)
{
  return PyInt_FromLong(((IplImage*)(cva->a))->origin);
}
static void dcam_setorigin(dcam_t *cva, PyObject *v)
{
  ((IplImage*)(cva->a))->origin = PyInt_AsLong(v);
}

static PyGetSetDef dcam_getseters[] = {
  {(char*)"nChannels", (getter)dcam_getnChannels, (setter)NULL, (char*)"nChannels", NULL},
  {(char*)"width", (getter)dcam_getwidth, (setter)NULL, (char*)"width", NULL},
  {(char*)"height", (getter)dcam_getheight, (setter)NULL, (char*)"height", NULL},
  {(char*)"depth", (getter)dcam_getdepth, (setter)NULL, (char*)"depth", NULL},
  {(char*)"origin", (getter)dcam_getorigin, (setter)dcam_setorigin, (char*)"origin", NULL},
  {NULL}  /* Sentinel */
};

static PyMappingMethods dcam_as_map = {
  NULL,
  &cvarr_GetItem,
  &cvarr_SetItem,
};
#endif

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

    assert(stIm->imLeft->imRect != NULL);
    assert(stIm->imRight->imRect != NULL);

    return Py_BuildValue("iiNN",
                         w,
                         h,
                         PyBuffer_FromMemory(stIm->imLeft->imRect, w * h),
                         PyBuffer_FromMemory(stIm->imRight->imRect, w * h));
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

  // This code copied from ost.cpp - it is the initialization sequence
  // for the camera.
  //
  int devIndex = 0;
  ps->dev = new dcam::StereoDcam(dcam::getGuid(devIndex));
  ps->dev->setRangeMax(4.0);	// in meters
  ps->dev->setRangeMin(0.5);	// in meters
  static videre_proc_mode_t pmode = PROC_MODE_RECTIFIED;
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
  dcam::init();

  dcam_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  dcam_specials();
  if (PyType_Ready(&dcam_Type) < 0)
    return;
  Py_InitModule("dcam", methods);
}
