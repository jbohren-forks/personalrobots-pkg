
typedef struct {
    PyObject_HEAD
    BruteForceMatcher <SparseSignature, int>
#if 1
    *
#endif
    c; 
} wrapped_BruteForceMatcher_t;

static void
wrapped_BruteForceMatcher_dealloc(PyObject *self)
{
  wrapped_BruteForceMatcher_t *pc = (wrapped_BruteForceMatcher_t*)self;
#if 1
  delete pc->c;
#else
  pc->c.~BruteForceMatcher();
#endif
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
#if 1
    object->c = new BruteForceMatcher <SparseSignature, int>;
#endif
    return (PyObject*)object;
}


typedef struct {
    PyObject_HEAD
    SparseSignature 
#if 0
    *
#endif
    c; 
} wrapped_SparseSignature_t;

static void
wrapped_SparseSignature_dealloc(PyObject *self)
{
  wrapped_SparseSignature_t *pc = (wrapped_SparseSignature_t*)self;
#if 0
  delete pc->c;
#else
  pc->c.~SparseSignature();
#endif
  PyObject_Del(self);
}

PyObject *wrapped_SparseSignature_dump(PyObject *self, PyObject *args);

/* Method table */
static PyMethodDef wrapped_SparseSignature_methods[] = {
{"dump", wrapped_SparseSignature_dump, METH_VARARGS},
{NULL, NULL}
};

static PyObject *
wrapped_SparseSignature_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(wrapped_SparseSignature_methods, self, attrname);
}

static PyTypeObject wrapped_SparseSignature_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "SparseSignature",
    sizeof(wrapped_SparseSignature_t),
    0,
    (destructor)wrapped_SparseSignature_dealloc,
    0,
    (getattrfunc)wrapped_SparseSignature_GetAttr,
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

PyObject *make_wrapped_SparseSignature(PyObject *self, PyObject *args)
{
    wrapped_SparseSignature_t *object = PyObject_NEW(wrapped_SparseSignature_t, &wrapped_SparseSignature_Type);
#if 0
    object->c = new SparseSignature ;
#endif
    return (PyObject*)object;
}

