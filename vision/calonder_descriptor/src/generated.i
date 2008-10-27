
typedef struct {
    PyObject_HEAD
    BruteForceMatcher <DenseSignature, PyObject *>
#if 1
    *
#endif
    c; 
} wrapped_BruteForceMatcher_t;

static void
wrapped_BruteForceMatcher_dealloc(PyObject *self)
{
#if 1
  wrapped_BruteForceMatcher_t *pc = (wrapped_BruteForceMatcher_t*)self;
  delete pc->c;
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
    object->c = new BruteForceMatcher <DenseSignature, PyObject *>;
#endif
    return (PyObject*)object;
}


typedef struct {
    PyObject_HEAD
    DenseSignature 
#if 0
    *
#endif
    c; 
} wrapped_DenseSignature_t;

static void
wrapped_DenseSignature_dealloc(PyObject *self)
{
#if 0
  wrapped_DenseSignature_t *pc = (wrapped_DenseSignature_t*)self;
  delete pc->c;
#endif
  PyObject_Del(self);
}

PyObject *wrapped_DenseSignature_dump(PyObject *self, PyObject *args);

/* Method table */
static PyMethodDef wrapped_DenseSignature_methods[] = {
{"dump", wrapped_DenseSignature_dump, METH_VARARGS},
{NULL, NULL}
};

static PyObject *
wrapped_DenseSignature_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(wrapped_DenseSignature_methods, self, attrname);
}

static PyTypeObject wrapped_DenseSignature_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "DenseSignature",
    sizeof(wrapped_DenseSignature_t),
    0,
    (destructor)wrapped_DenseSignature_dealloc,
    0,
    (getattrfunc)wrapped_DenseSignature_GetAttr,
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

PyObject *make_wrapped_DenseSignature(PyObject *self, PyObject *args)
{
    wrapped_DenseSignature_t *object = PyObject_NEW(wrapped_DenseSignature_t, &wrapped_DenseSignature_Type);
#if 0
    object->c = new DenseSignature ;
#endif
    return (PyObject*)object;
}

