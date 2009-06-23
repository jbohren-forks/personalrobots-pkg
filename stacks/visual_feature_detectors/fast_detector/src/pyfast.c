#include "Python.h"
#include "fast.h"

static inline int corner_score(const byte*  imp, const int *pointer_dir, int barrier)
{
	/*The score for a positive feature is sum of the difference between the pixels
	  and the barrier if the difference is positive. Negative is similar.
	  The score is the max of those two.
	  
	   B = {x | x = points on the Bresenham circle around c}
	   Sp = { I(x) - t | x E B , I(x) - t > 0 }
	   Sn = { t - I(x) | x E B, t - I(x) > 0}
	  
	   Score = max sum(Sp), sum(Sn)*/

	int cb = *imp + barrier;
	int c_b = *imp - barrier;
	int sp=0, sn = 0;

#if 0
	for(i=0; i<16; i++)
	{
		int p = imp[pointer_dir[i]];

		if(p > cb)
			sp += p-cb;
		else if(p < c_b)
			sn += c_b-p;
	}
#else
{
  int p;
#define ACC(i) p = imp[pointer_dir[i]]; if (p > cb) sp += p-cb; else sn += c_b-p;
        ACC(0)
        ACC(1)
        ACC(2)
        ACC(3)
        ACC(4)
        ACC(5)
        ACC(6)
        ACC(7)
        ACC(8)
        ACC(9)
        ACC(10)
        ACC(11)
        ACC(12)
        ACC(13)
        ACC(14)
        ACC(15)
#undef ACC
}
#endif
	
	if(sp > sn)
		return sp;
	else 
		return sn;
}

PyObject *fast(PyObject *self, PyObject *args)
{
    int imdata_size;
    char *imdata;
    int xsize, ysize, barrier, threshold;
    if (!PyArg_ParseTuple(args, "s#iiii", &imdata, &imdata_size, &xsize, &ysize, &barrier, &threshold))
        return NULL;

    int numcorners = 0, num_nonmax = 0;
    xyr *corners = fast_corner_detect_9(imdata, xsize, ysize, barrier, &numcorners);

    int	pixel[16];
    pixel[0] = 0 + 3 * xsize;		
    pixel[1] = 1 + 3 * xsize;		
    pixel[2] = 2 + 2 * xsize;		
    pixel[3] = 3 + 1 * xsize;		
    pixel[4] = 3 + 0 * xsize;		
    pixel[5] = 3 + -1 * xsize;		
    pixel[6] = 2 + -2 * xsize;		
    pixel[7] = 1 + -3 * xsize;		
    pixel[8] = 0 + -3 * xsize;		
    pixel[9] = -1 + -3 * xsize;		
    pixel[10] = -2 + -2 * xsize;		
    pixel[11] = -3 + -1 * xsize;		
    pixel[12] = -3 + 0 * xsize;		
    pixel[13] = -3 + 1 * xsize;		
    pixel[14] = -2 + 2 * xsize;		
    pixel[15] = -1 + 3 * xsize;		

    PyObject *r = PyList_New(0);
    int i;
    for (i = 0; i < numcorners; i++) {
      int x = corners[i].x, y = corners[i].y;
      int s = corner_score(imdata + x + y * xsize, pixel, barrier);
      if (s > threshold) {
        PyObject *t = PyTuple_New(3);
        PyTuple_SetItem(t, 0, PyInt_FromLong(x));
        PyTuple_SetItem(t, 1, PyInt_FromLong(y));
        PyTuple_SetItem(t, 2, PyInt_FromLong(s));
        PyList_Append(r, t);
        Py_DECREF(t);
      }
    }
    free(corners);

    return r;
}

PyObject *nonmax(PyObject *self, PyObject *args)
{
  PyObject *ppts;
  Py_ssize_t i, j;
  int xi, yi, ri;
  int xj, yj, rj;
  int dist2;

  if (!PyArg_ParseTuple(args, "O", &ppts))
    return NULL;
  Py_ssize_t sz = PyList_Size(ppts);

  char suppress[sz];
  for (i = 0; i < sz; i++)
    suppress[i] = 0;
  xyr pts[sz];

  for (i = 0; i < sz; i++) {
    PyObject *t = PyList_GET_ITEM(ppts, i);
    pts[i].x = PyInt_AsLong(PyTuple_GET_ITEM(t, 0));
    pts[i].y = PyInt_AsLong(PyTuple_GET_ITEM(t, 1));
    pts[i].r = PyInt_AsLong(PyTuple_GET_ITEM(t, 2));
  }

  for (i = 0; i < sz; i++) {
    xi = pts[i].x;
    yi = pts[i].y;
    ri = pts[i].r;
    for (j = i + 1; j < sz; j++) {
      yj = pts[j].y;
      if ((yj - yi) > 1) {
        continue;
      }
      rj = pts[j].r;
      xj = pts[j].x;
      dist2 = (xi-xj)*(xi-xj) + (yi-yj)*(yi-yj);
      if (dist2 < 2) {
        if (ri < rj)
          suppress[i] = 1;
        else
          suppress[j] = 1;
      }
    }
  }

  PyObject *r = PyList_New(0);
  for (i = 0; i < sz; i++) {
    if (!suppress[i]) {
      PyObject *t = PyList_GET_ITEM(ppts, i);
      PyList_Append(r, t);
    }
  }

  return r;
}

static PyMethodDef methods[] = {
  {"fast", fast, METH_VARARGS},
  {"nonmax", nonmax, METH_VARARGS},
  {NULL, NULL},
};

void initfast()
{
    PyObject *m, *d;

    m = Py_InitModule("fast", methods);
    d = PyModule_GetDict(m);
}
