#include "Python.h"

#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <highgui.h>
#include <cvwimage.h>
#include <cstdio>
#include <ctime>

using namespace features;

#include "generated.i"

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

PyObject *setThreshold(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;
  double thresh;
  if (!PyArg_ParseTuple(args, "d", &thresh)) return NULL;
  pc->classifier->setThreshold(thresh);
  Py_RETURN_NONE;
}

PyObject *train(PyObject *self, PyObject *args)
{
  classifier_t *pc = (classifier_t*)self;

  IplImage *input;
  PyObject *kp;

  {
    char *imgdata;
    int imgdata_size, x, y;
    if (!PyArg_ParseTuple(args, "s#iiO", &imgdata, &imgdata_size, &x, &y, &kp))
      return NULL;
    input = cvCreateImageHeader(cvSize(x, y), IPL_DEPTH_8U, 1);
    cvSetData(input, imgdata, x);
  }
  std::vector<BaseKeypoint> base_set;
  for (int i = 0; i < PyList_Size(kp); i++) {
    int x, y;
    PyArg_ParseTuple(PyList_GetItem(kp, i), "ii", &x, &y);
    base_set.push_back( BaseKeypoint(x+16, y+16, input) );
  }

  Rng rng( 0 );
  //pc->classifier->train(base_set, rng, 25, 10, 20);
  pc->classifier->train(base_set, rng, 25, 10, 1000);

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

PyObject *getSparseSignature(PyObject *self, PyObject *args)
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
  wrapped_SparseSignature_t *object = PyObject_NEW(wrapped_SparseSignature_t, &wrapped_SparseSignature_Type);
  //new(&object->c) SparseSignature();
  new(&object->c) SparseSignature(pc->classifier->getSparseSignature(input));
  //object->c = pc->classifier->getSparseSignature(input);
  //SparseSignature sig = pc->classifier->getSparseSignature(input);
  //object->c.assign_temporary(sig);
  return (PyObject*)object;
}

/* Method table */
static PyMethodDef classifier_methods[] = {
  {"setThreshold", setThreshold, METH_VARARGS},
  {"train", train, METH_VARARGS},
  {"write", Cwrite, METH_VARARGS},
  {"read", Cread, METH_VARARGS},
  {"getSparseSignature", getSparseSignature, METH_VARARGS},
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

  // Py_INCREF(tag);

  wrapped_SparseSignature_t *ps = (wrapped_SparseSignature_t*)sig;
  pm->c->addSignature(ps->c, 0);
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
  wrapped_SparseSignature_t *ps = (wrapped_SparseSignature_t*)sig;

  float distance;
  int index;
  if (predicates == NULL)
    index = pm->c->findMatch(ps->c, &distance);
  else {
    index = pm->c->findMatchPredicated(ps->c, predicates, &distance);
  }
  if (index == -1)
    Py_RETURN_NONE;
  else
    return Py_BuildValue("id", index, distance);
}

PyObject *wrapped_SparseSignature_dump(PyObject *self, PyObject *args)
{
  wrapped_SparseSignature_t *ps = (wrapped_SparseSignature_t*)self;

  PyObject *r = PyTuple_New(ps->c.size());
  for (size_t i = 0; i < ps->c.size(); i++) {
    PyTuple_SetItem(r, i, PyFloat_FromDouble(ps->c[i]));
  }
  return r;
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
    object->classifier = new RTreeClassifier;
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
