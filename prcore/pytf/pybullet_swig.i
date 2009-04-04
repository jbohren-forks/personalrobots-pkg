 %module pybullet_swig
 %{
 /* Includes the header in the wrapper code */
#include "pybtVector3.h"
#include "pybtQuaternion.h"
// #include "LinearMath/btTransform.h"
 %}
 
%include "std_string.i"

%include "LinearMath/btScalar.h"
%include "LinearMath/btMinMax.h"


%include "pybtVector3.h"
%include "pybtQuaternion.h"
//%include "pybtMatrix3x3.h"
//%include "pybtTransform.h"

%extend py::btQuaternion{
  char * __str__()
  {
    static char temp[256];
    sprintf(temp, "[%f, %f, %f, %f]", self->getX(), self->getY(), self->getZ(),  self->getW());
    return &temp[0];
  };
  
  btQuaternion  __rmul__(const btVector3& w)
{
  py::btQuaternion& q = *self;
  return py::btQuaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
};


 }

%extend py::btVector3 {
  char * __str__()
  {
    static char temp[256];
    sprintf(temp, "[%f, %f, %f]", self->getX(), self->getY(), self->getZ());
    return &temp[0];
  };
  

   btVector3 __rmul__(float s) 
  {
    return py::btVector3(self->m_floats[0] * s, self->m_floats[1] * s, self->m_floats[2] * s);
  };

  btVector3 __rdiv__(float s) 
  {
    btFullAssert(s != btScalar(0.0));
    return self->operator*(btScalar(1.0) / s);
  };

 }
