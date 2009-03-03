#include <Python.h>

#include <iostream>
#include <vector>
#include <queue>

using namespace std;

#include <treeoptimizer3.hh>

using namespace AISNavigation;

typedef TreeOptimizer3::Transformation Transformation;
typedef TreeOptimizer3::Covariance Covariance;
typedef TreeOptimizer3::Information Information;
typedef TreeOptimizer3::Translation Translation;
typedef TreeOptimizer3::Rotation Rotation;
typedef TreeOptimizer3::Pose Pose;
typedef TreeOptimizer3::Vertex Vertex;
typedef TreeOptimizer3::Edge Edge;

typedef struct {
  PyObject_HEAD
  TreeOptimizer3 *to;
} treeoptimizer3_t;

PyObject *pginitializeOnlineOptimization(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  to->initializeOnlineOptimization();
  Py_RETURN_NONE;
}

PyObject *pginitializeOnlineIterations(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  to->initializeOnlineIterations();
  Py_RETURN_NONE;
}

PyObject *pgiterate(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  to->iterate();
  Py_RETURN_NONE;
}

PyObject *pgrecomputeAllTransformations(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  to->recomputeAllTransformations();
  Py_RETURN_NONE;
}

PyObject *pgremoveEdge(PyObject *self, PyObject *args)
{
  int i0, i1;

  if (!PyArg_ParseTuple(args, "ii", &i0, &i1))
    return NULL;

  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  Edge *e = to->edge(i0, i1);
  to->removeEdge(e);
  Py_RETURN_NONE;
}

PyObject *pgaddIncrementalEdge(PyObject *self, PyObject *args)
{

  static const double CLOSURE_XY_SIGMA = 0.01;
  static const double CLOSURE_Z_SIGMA = 0.01;
  static const double CLOSURE_ROLL_SIGMA = 0.0002;
  static const double CLOSURE_PITCH_SIGMA = 0.0002;
  static const double CLOSURE_YAW_SIGMA = 0.002;

  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  int i0, i1;
  double x, y, z;
  double roll, pitch, yaw;
  double a, b, c, d, e, f;

  a = 1.0 / CLOSURE_XY_SIGMA;
  b = 1.0 / CLOSURE_XY_SIGMA;
  c = 1.0 / CLOSURE_Z_SIGMA;
  d = 1.0 / CLOSURE_ROLL_SIGMA;
  e = 1.0 / CLOSURE_PITCH_SIGMA;
  f = 1.0 / CLOSURE_YAW_SIGMA;

  Information inf(6,6);
  if (!PyArg_ParseTuple(args, "ii(ddd)(ddd)|(dddddd)", &i0, &i1, &x, &y, &z, &roll, &pitch, &yaw, &a, &b, &c, &d, &e, &f))
    return NULL;
  // Transformation t(Translation(0.0, 0.0, 0.0), Rotation(0.0, 0.0, 0.0, 0.0));
  Transformation t(x, y, z, roll, pitch, yaw);

  // Set covariance for relative poses from VO and loop closures
  inf[0][0] = a;
  inf[1][1] = b;
  inf[2][2] = c;
  inf[3][3] = d;
  inf[4][4] = e;
  inf[5][5] = f;

  to->addIncrementalEdge(i0, i1, t, inf);

  Py_RETURN_NONE;
}

PyObject *pgerror(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  return PyFloat_FromDouble(to->error());
}

PyObject *pgvertex(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  int id;
  if (!PyArg_ParseTuple(args, "i", &id))
    return NULL;
  Vertex* pv = to->vertex(id);
  printf("Vertex[%d] %p has parent %p\n", id, pv, pv->parent);
  if (pv == NULL) {
    PyErr_SetString(PyExc_TypeError, "no such vertex");
    return NULL;
  }
  Pose pp = pv->transformation.toPoseType();
  return Py_BuildValue("(ddd)(ddd)", pp.x(), pp.y(), pp.z(), pp.roll(), pp.pitch(), pp.yaw());
}

PyObject *pgsave(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename))
    return NULL;
  to->save(filename);
  Py_RETURN_NONE;
}

PyObject *pgload(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename))
    return NULL;
  to->load(filename);
  Py_RETURN_NONE;
}

PyObject *pgclear(PyObject *self, PyObject *args)
{
  TreeOptimizer3 *to = ((treeoptimizer3_t*)self)->to;
  to->clear();
  Py_RETURN_NONE;
}

/* Method table */
static PyMethodDef treeoptimizer3_methods[] = {
  {"initializeOnlineOptimization", pginitializeOnlineOptimization, METH_VARARGS},
  {"save", pgsave, METH_VARARGS},
  {"load", pgload, METH_VARARGS},
  {"addIncrementalEdge", pgaddIncrementalEdge, METH_VARARGS },
  {"removeEdge", pgremoveEdge, METH_VARARGS },
  {"initializeOnlineIterations", pginitializeOnlineIterations, METH_VARARGS},
  {"clear", pgclear, METH_VARARGS},
  {"error", pgerror, METH_VARARGS},
  {"iterate", pgiterate, METH_VARARGS},
  {"recomputeAllTransformations", pgrecomputeAllTransformations, METH_VARARGS},
  {"vertex", pgvertex, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
treeoptimizer3_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(treeoptimizer3_methods, self, attrname);
}

static void
treeoptimizer3_dealloc(PyObject *self)
{
  PyObject_Del(self);
}

static PyTypeObject treeoptimizer3_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "TreeOptimizer3",
    sizeof(treeoptimizer3_t),
    0,
    (destructor)treeoptimizer3_dealloc,
    0,
    (getattrfunc)treeoptimizer3_GetAttr,
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

PyObject *mktreeoptimizer3(PyObject *self, PyObject *args)
{
  treeoptimizer3_t *object = PyObject_NEW(treeoptimizer3_t, &treeoptimizer3_Type);
  object->to = new TreeOptimizer3();
  object->to->verboseLevel = 0;
  object->to->restartOnDivergence = false;

  return (PyObject*)object;
}

static PyMethodDef methods[] = {
  {"TreeOptimizer3", mktreeoptimizer3, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void initpytoro()
{
    PyObject *m, *d;

    m = Py_InitModule("pytoro", methods);
    d = PyModule_GetDict(m);
}
