#include "Python.h"
#include "fast.h"

PyObject *fast(PyObject *self, PyObject *args)
{
    unsigned char *imdata = (unsigned char*)PyString_AsString(PyTuple_GetItem(args,0));
    int xsize = PyInt_AsLong(PyTuple_GetItem(args, 1));
    int ysize = PyInt_AsLong(PyTuple_GetItem(args, 2));
    int threshold = PyInt_AsLong(PyTuple_GetItem(args, 3));
    int barrier = PyInt_AsLong(PyTuple_GetItem(args, 4));
    int numcorners, num_nonmax;
    xy *corners = fast_corner_detect_9(imdata, xsize, ysize, threshold, &numcorners);

#if 1
    xy *nm = fast_nonmax(imdata, xsize, ysize, corners, numcorners, barrier, &num_nonmax);
#else
    xy *nm = corners;
    num_nonmax = numcorners;
#endif
    PyObject *r = PyList_New(num_nonmax);
    int i;

    for (i = 0; i < num_nonmax; i++) {
        PyObject *t = PyTuple_New(2);
        PyTuple_SetItem(t, 0, PyInt_FromLong(nm[i].x));
        PyTuple_SetItem(t, 1, PyInt_FromLong(nm[i].y));
        PyList_SetItem(r, i, t);
    }
    if (corners)
        free(corners);
#if 0
    if (nm)
        free(nm);
#endif

    return r;
}

static PyMethodDef methods[] = {
  {"fast", fast, METH_VARARGS},
  {NULL, NULL},
};

void initfast()
{
    PyObject *m, *d;

    m = Py_InitModule("fast", methods);
    d = PyModule_GetDict(m);
}
