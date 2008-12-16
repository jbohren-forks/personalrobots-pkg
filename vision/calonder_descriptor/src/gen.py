template0 = """
typedef struct {
    PyObject_HEAD
    XXX %INIT%
#if %PTR%
    *
#endif
    c; 
} wrapped_XXX_t;

static void
wrapped_XXX_dealloc(PyObject *self)
{
  wrapped_XXX_t *pc = (wrapped_XXX_t*)self;
#if %PTR%
  delete pc->c;
#else
  pc->c.~XXX();
#endif
  PyObject_Del(self);
}

%METHDECLS%

/* Method table */
static PyMethodDef wrapped_XXX_methods[] = {
%METHLINES%
{NULL, NULL}
};

static PyObject *
wrapped_XXX_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(wrapped_XXX_methods, self, attrname);
}

static PyTypeObject wrapped_XXX_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "XXX",
    sizeof(wrapped_XXX_t),
    0,
    (destructor)wrapped_XXX_dealloc,
    0,
    (getattrfunc)wrapped_XXX_GetAttr,
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

PyObject *make_wrapped_XXX(PyObject *self, PyObject *args)
{
    wrapped_XXX_t *object = PyObject_NEW(wrapped_XXX_t, &wrapped_XXX_Type);
#if %PTR%
    object->c = new XXX %INIT%;
#endif
    return (PyObject*)object;
}
"""

o = {
#   'SparseSignature' : { 'meths' : [ 'dump' ] },
   'BruteForceMatcher' : { 'init' : '<int>', 'meths' : ['addSignature', 'findMatch'] }
}

import re
op = open("src/generated.i", "w")
for nm,props in o.items():

   if 'init' in props:
     ptr = 1
   else:
     ptr = 0

   methdecls = "\n".join(["PyObject *wrapped_%s_%s(PyObject *self, PyObject *args);" % (nm, me) for me in props.get('meths', [])])
   methlines = "\n".join(["{\"%s\", wrapped_%s_%s, METH_VARARGS}," % (me, nm, me) for me in props.get('meths', [])])
   t = template0
   t = re.sub('%PTR%', str(ptr), t)
   t = re.sub('%METHDECLS%', methdecls, t)
   t = re.sub('%METHLINES%', methlines, t)
   t = re.sub('%INIT%', props.get('init', ''), t)
   t = re.sub('XXX', nm, t)
   print >>op, t
op.close()
