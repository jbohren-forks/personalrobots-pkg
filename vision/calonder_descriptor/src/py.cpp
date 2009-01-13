#include "Python.h"

#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"

#include <boost/foreach.hpp>
#include <highgui.h>
#include <cvwimage.h>
#include <algorithm>
#include <cstdlib>
#include <ctime>

using namespace features;

//typedef float SigType;
typedef uint8_t SigType;
typedef Promote<SigType>::type DistanceType;

typedef struct {
  PyObject_HEAD
  BruteForceMatcher <SigType, int> *c;
  std::vector<PyObject*> *sigs;
} wrapped_BruteForceMatcher_t;

static void
wrapped_BruteForceMatcher_dealloc(PyObject *self)
{
  wrapped_BruteForceMatcher_t *pc = (wrapped_BruteForceMatcher_t*)self;
  delete pc->c;
  BOOST_FOREACH(PyObject *sig, *pc->sigs)
    Py_DECREF(sig);
  delete pc->sigs;
  PyObject_Del(self);
}

PyObject *wrapped_BruteForceMatcher_addSignature(PyObject *self, PyObject *args);
PyObject *wrapped_BruteForceMatcher_findMatch(PyObject *self, PyObject *args);

/* Method table */
static PyMethodDef wrapped_BruteForceMatcher_methods[] = {
{"addSignature", wrapped_BruteForceMatcher_addSignature, METH_VARARGS},
{"findMatch", wrapped_BruteForceMatcher_findMatch, METH_VARARGS},
{NULL, NULL}
};

static PyObject *
wrapped_BruteForceMatcher_GetAttr(PyObject *self, char *attrname)
{
  return Py_FindMethod(wrapped_BruteForceMatcher_methods, self, attrname);
}

static PyTypeObject wrapped_BruteForceMatcher_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "BruteForceMatcher",
    sizeof(wrapped_BruteForceMatcher_t),
    0,
    (destructor)wrapped_BruteForceMatcher_dealloc,
    0,
    (getattrfunc)wrapped_BruteForceMatcher_GetAttr,
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

PyObject *make_wrapped_BruteForceMatcher(PyObject *self, PyObject *args)
{
  wrapped_BruteForceMatcher_t *object = PyObject_NEW(wrapped_BruteForceMatcher_t, &wrapped_BruteForceMatcher_Type);
  int dimension;
  if (!PyArg_ParseTuple(args, "i", &dimension))
    return NULL;
  object->c = new BruteForceMatcher <SigType, int>(dimension);
  object->sigs = new std::vector<PyObject*>;
  return (PyObject*)object;
}

typedef struct {
  PyObject_HEAD
  SigType *data;
  int size;
} signature_t;

static void
signature_dealloc(PyObject *self)
{
  signature_t *pc = (signature_t*)self;
  free(pc->data);
  PyObject_Del(self);
}

PyObject *signature_dump(PyObject *self, PyObject *args)
{
  signature_t *ps = (signature_t*)self;
  PyObject* list = PyList_New(0);
  for (int i = 0; i < ps->size; ++i)
    PyList_Append(list, PyFloat_FromDouble(ps->data[i]));
  return list;
}

/* Method table */
static PyMethodDef signature_methods[] = {
  {"dump", signature_dump, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
signature_GetAttr(PyObject *self, char *attrname)
{
  return Py_FindMethod(signature_methods, self, attrname);
}

static PyTypeObject signature_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "signature",
    sizeof(signature_t),
    0,
    (destructor)signature_dealloc,
    0,
    (getattrfunc)signature_GetAttr,
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

typedef struct {
  PyObject_HEAD
  RTreeClassifier *classifier;
} classifier_t;

static void
classifier_dealloc(PyObject *self)
{
  classifier_t *pc = (classifier_t*)self;
  delete pc->classifier;
  PyObject_Del(self);
}

// TODO: keyword arguments for training parameters
//       allow training on multiple images
PyObject *train(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;

  IplImage *input;
  PyObject *kp;
  int num_trees = RTreeClassifier::DEFAULT_TREES;
  int depth = RandomizedTree::DEFAULT_DEPTH;
  int views = RandomizedTree::DEFAULT_VIEWS;
  int dimension = RandomizedTree::DEFAULT_REDUCED_NUM_DIM;
  int num_quant_bits = 0;

  {
    char *imgdata;
    int imgdata_size, x, y;
    if (!PyArg_ParseTuple(args, "s#iiO|iiiii", &imgdata, &imgdata_size,
                          &x, &y, &kp, &num_trees, &depth, &views,
                          &dimension, &num_quant_bits))
      return NULL;
    dimension = std::min(dimension, PyList_Size(kp));
    input = cvCreateImageHeader(cvSize(x, y), IPL_DEPTH_8U, 1);
    cvSetData(input, imgdata, x);
  }
  std::vector<BaseKeypoint> base_set;
  for (int i = 0; i < PyList_Size(kp); i++) {
    int x, y;
    PyArg_ParseTuple(PyList_GetItem(kp, i), "ii", &x, &y);
    base_set.push_back( BaseKeypoint(x+16, y+16, input) );
  }

  //Rng rng( 0 );
  Rng rng( std::time(0) );
  pc->classifier->train(base_set, rng, num_trees, depth,
                        views, dimension, num_quant_bits);

  Py_RETURN_NONE;
}

PyObject *Cwrite(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename)) return NULL;
  pc->classifier->write(filename);
  Py_RETURN_NONE;
}

PyObject *Cread(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename))
    return NULL;
  pc->classifier->read(filename);
  Py_RETURN_NONE;
}

PyObject *getSignature(PyObject *self, PyObject *args)
{
  IplImage *input;
  {
    char *imgdata;
    int imgdata_size, x, y;
    if (!PyArg_ParseTuple(args, "s#ii", &imgdata, &imgdata_size, &x, &y))
      return NULL;
    input = cvCreateImageHeader(cvSize(x, y), IPL_DEPTH_8U, 1);
    cvSetData(input, imgdata, x);
  }
  classifier_t *pc = (classifier_t*)self;
  signature_t *object = PyObject_NEW(signature_t, &signature_Type);
  object->size = pc->classifier->classes();
  posix_memalign((void**)&object->data, 16, object->size * sizeof(SigType));
  pc->classifier->getSignature(input, object->data);
  return (PyObject*)object;
}

static int im2arr(CvArr **dst, PyObject *src)
{
  int width, height;
  if (!PyObject_HasAttrString(src, "size") ||
      !PyObject_HasAttrString(src, "mode") ||
      !PyObject_HasAttrString(src, "tostring"))
    return 0;

  if (!PyArg_ParseTuple(PyObject_GetAttrString(src, "size"), "ii", &width, &height))
    return 0;

  char *mode = PyString_AsString(PyObject_GetAttrString(src, "mode"));
  int depth, nchannels, bps;
  nchannels = (int)strlen(mode);
  if (strcmp(mode, "F") == 0) {
    depth = IPL_DEPTH_32F;
    bps = 4;
  } else {
    depth = IPL_DEPTH_8U;
    bps = 1;
  }

  char *string;
  Py_ssize_t length;
  PyString_AsStringAndSize(PyObject_CallObject(PyObject_GetAttrString(src, "tostring"), NULL), &string, &length);

  *dst = cvCreateImageHeader(cvSize(width, height), depth, nchannels);
  cvSetData(*dst, (void*)string, nchannels * bps * width);

  return 1;
}

PyObject *getSignatures(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;

  PyObject *pim, *coords;
  if (!PyArg_ParseTuple(args, "OO", &pim, &coords))
    return NULL;

  IplImage *cva;
  IplImage subrect;
  if (!im2arr((CvArr**)&cva, pim)) assert(0);

  PyObject *r = PyList_New(0);

  IplImage *input = cvCreateImage(cvSize(32, 32), IPL_DEPTH_8U, 1);

  PyObject *iterator = PyObject_GetIter(coords);
  PyObject *t;
  assert(iterator != NULL);
  while ((t = PyIter_Next(iterator)) != NULL) {
    int x, y;
    if (!PyArg_ParseTuple(t, "ii", &x, &y))
        return NULL;
    // cvGetSubRect(cva, (CvMat*)&subrect, cvRect(x - 16, y - 16, 32, 32));
    // cvCopy(&subrect, input);

    cvSetImageROI(cva, cvRect(x - 16, y - 16, 32, 32));
    cvCopy(cva, input);

    signature_t *object = PyObject_NEW(signature_t, &signature_Type);
    object->size = pc->classifier->classes();
    posix_memalign((void**)&object->data, 16, object->size * sizeof(float));
    pc->classifier->getSignature(input, object->data);
    PyList_Append(r, (PyObject*)object);
  }
  return r;
}

/* Method table */
static PyMethodDef classifier_methods[] = {
  {"train", train, METH_VARARGS},
  {"write", Cwrite, METH_VARARGS},
  {"read", Cread, METH_VARARGS},
  {"getSignature", getSignature, METH_VARARGS},
  {"getSignatures", getSignatures, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
classifier_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(classifier_methods, self, attrname);
}

PyObject *wrapped_BruteForceMatcher_addSignature(PyObject *self, PyObject *args)
{
  wrapped_BruteForceMatcher_t *pm = (wrapped_BruteForceMatcher_t*)self;

  PyObject *sig;
  if (!PyArg_ParseTuple(args, "O", &sig))
    return NULL;

  signature_t *ps = (signature_t*)sig;
  pm->c->addSignature(ps->data, 0);
  // claim reference to ensure sig isn't deleted prematurely!
  Py_INCREF(sig);
  pm->sigs->push_back(sig);
  Py_RETURN_NONE;
}

PyObject *wrapped_BruteForceMatcher_findMatch(PyObject *self, PyObject *args)
{
  wrapped_BruteForceMatcher_t *pm = (wrapped_BruteForceMatcher_t*)self;

  PyObject *sig;
  int predicates_size;
  char *predicates = NULL;

  if (!PyArg_ParseTuple(args, "O|s#", &sig, &predicates, &predicates_size))
    return NULL;
  signature_t *ps = (signature_t*)sig;

  DistanceType distance;
  int index;
  if (predicates == NULL)
    index = pm->c->findMatch(ps->data, &distance);
  else {
    index = pm->c->findMatchPredicated(ps->data, predicates, &distance);
  }

  if (index == -1)
    Py_RETURN_NONE;
  else
    return Py_BuildValue("id", index, distance);
}

static PyTypeObject classifier_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "classifier",
    sizeof(classifier_t),
    0,
    (destructor)classifier_dealloc,
    0,
    (getattrfunc)classifier_GetAttr,
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

PyObject *classifier(PyObject *self, PyObject *args)
{
    classifier_t *object = PyObject_NEW(classifier_t, &classifier_Type);
    object->classifier = new RTreeClassifier(true);
    return (PyObject*)object;
}

static PyMethodDef methods[] = {
  {"classifier", classifier, METH_VARARGS},
  {"BruteForceMatcher", make_wrapped_BruteForceMatcher, METH_VARARGS},
  {NULL, NULL},
};

extern "C" void initcalonder()
{
    PyObject *m, *d;

    m = Py_InitModule("calonder", methods);
    d = PyModule_GetDict(m);
}
