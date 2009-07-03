#include <Python.h>

#include "tf/tf.h"

struct transformer_t {
  PyObject_HEAD
  tf::Transformer *t;
};

static PyTypeObject transformer_Type = {
  PyObject_HEAD_INIT(&PyType_Type)
  0,                               /*size*/
  "tf.transformer",                /*name*/
  sizeof(transformer_t),           /*basicsize*/
};

static PyObject *PyObject_StealAttrString(PyObject* o, const char *name)
{
    PyObject *r = PyObject_GetAttrString(o, name);
    if (r != NULL)
      Py_DECREF(r);
    return r;
}

static PyObject *mkTransformer(PyObject *self, PyObject *args)
{
  int interpolating = 1;
  PyObject *py_cache_time = NULL;

  if (!PyArg_ParseTuple(args, "|iO", &interpolating, &py_cache_time))
    return NULL;

  transformer_t *r = PyObject_NEW(transformer_t, &transformer_Type);

  ros::Duration cache_time;
  if (py_cache_time != NULL) {
    PyObject *tsr = PyObject_CallMethod(py_cache_time, "to_seconds", NULL);
    if (tsr == NULL) {
       PyErr_SetString(PyExc_TypeError, "'cache_time' must have a to_seconds method, e.g. rospy.Duration");
       return NULL;
    }
    cache_time.fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
  } else {
    cache_time.fromSec(tf::Transformer::DEFAULT_CACHE_TIME);
  }

  r->t = new tf::Transformer(interpolating, cache_time);

  return (PyObject*)r;
}

static PyObject *allFramesAsString(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->allFramesAsString().c_str());
}

static PyObject *setTransform(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  PyObject *py_transform;
  char *authority = (char*)"default_authority";

  if (!PyArg_ParseTuple(args, "O|s", &py_transform, &authority))
    return NULL;
  tf::Stamped< btTransform > transform;
  PyObject *header = PyObject_StealAttrString(py_transform, "header");
  transform.frame_id_ = PyString_AsString(PyObject_StealAttrString(header, "frame_id"));
  transform.parent_id_ = PyString_AsString(PyObject_StealAttrString(py_transform, "parent_id"));
  PyObject *tsr = PyObject_CallMethod(PyObject_StealAttrString(header, "stamp"), "to_seconds", NULL);
  if (tsr == NULL) {
     PyErr_SetString(PyExc_TypeError, "'stamp' must have a to_seconds method, e.g. rospy.Time");
     return NULL;
  }
  transform.stamp_ = ros::Time().fromSec(PyFloat_AsDouble(tsr));
  Py_DECREF(tsr);

  PyObject *mtransform = PyObject_StealAttrString(py_transform, "transform");
  PyObject *translation = PyObject_StealAttrString(mtransform, "translation");
  double tx = PyFloat_AsDouble(PyObject_StealAttrString(translation, "x"));
  double ty = PyFloat_AsDouble(PyObject_StealAttrString(translation, "y"));
  double tz = PyFloat_AsDouble(PyObject_StealAttrString(translation, "z"));
  PyObject *rotation = PyObject_StealAttrString(mtransform, "rotation");
  double qx = PyFloat_AsDouble(PyObject_StealAttrString(rotation, "x"));
  double qy = PyFloat_AsDouble(PyObject_StealAttrString(rotation, "y"));
  double qz = PyFloat_AsDouble(PyObject_StealAttrString(rotation, "z"));
  double qw = PyFloat_AsDouble(PyObject_StealAttrString(rotation, "w"));

  transform.setData(btTransform(
    btQuaternion(btScalar(qx), btScalar(qy), btScalar(qz), btScalar(qw)),
    btVector3(btScalar(tx), btScalar(ty), btScalar(tz))));
  t->setTransform(transform, authority);
  Py_RETURN_NONE;
}

static struct PyMethodDef transformer_methods[] =
{
  {"allFramesAsString", allFramesAsString, METH_VARARGS},
  {"setTransform", setTransform, METH_VARARGS},
  {NULL,          NULL}
};

static PyMethodDef module_methods[] = {
  {"Transformer", mkTransformer, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void init_tfX()
{
  PyObject *m, *d;

  transformer_Type.tp_alloc = PyType_GenericAlloc;
  transformer_Type.tp_new = PyType_GenericNew;
  transformer_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  transformer_Type.tp_methods = transformer_methods;
  if (PyType_Ready(&transformer_Type) != 0)
    return;

  m = Py_InitModule("_tfX", module_methods);
  d = PyModule_GetDict(m);
}

