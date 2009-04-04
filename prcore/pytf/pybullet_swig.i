 %module pybullet_swig
 %{
 /* Includes the header in the wrapper code */
 #include "pybtVector3.h"
// #include "LinearMath/btTransform.h"
 %}
 
%include "std_string.i"

%include "LinearMath/btScalar.h"
%include "LinearMath/btMinMax.h"


%include "pybtVector3.h"
//%include "pybtMatrix3x3.h"
//%include "pybtTransform.h"

%extend btVector3 {
  char * __str__()
  {
    static char temp[256];
    sprintf(temp, "[%f, %f, %f]", self->getX(), self->getY(), self->getZ());
    return &temp[0];
  };
  

   btVector3 __rmul__(float s) 
  {
    return btVector3(self->m_floats[0] * s, self->m_floats[1] * s, self->m_floats[2] * s);
  };

  btVector3 __rdiv__(float s) 
  {
    btFullAssert(s != btScalar(0.0));
    return self->operator*(btScalar(1.0) / s);
  };

 }
