 %module pybullet_swig
 %{
 /* Includes the header in the wrapper code */
 #include "LinearMath/btTransform.h"
 %}
 
%include "std_string.i"


%include "LinearMath/btScalar.h"
%include "LinearMath/btMinMax.h"

%include "pybtVector3.h"
%include "pybtMatrix3x3.h"
%include "pybtTransform.h"

