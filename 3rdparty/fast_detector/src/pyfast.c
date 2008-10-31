#include "Python.h"
#include "fast.h"

PyObject *fast(PyObject *self, PyObject *args)
{
    int imdata_size;
    char *imdata;
    int xsize, ysize, threshold, barrier;
    if (!PyArg_ParseTuple(args, "s#iiii", &imdata, &imdata_size, &xsize, &ysize, &threshold, &barrier))
        return NULL;

    int numcorners = 0, num_nonmax = 0;
    xy *corners = fast_corner_detect_9(imdata, xsize, ysize, threshold, &numcorners);


    xy *nm = fast_nonmax(imdata, xsize, ysize, corners, numcorners, barrier, &num_nonmax);
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
    if (nm)
        free(nm);

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
