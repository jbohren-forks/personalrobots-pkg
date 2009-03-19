 %module pybullet_swig
 %{
 /* Includes the header in the wrapper code */
 #include "LinearMath/btTransform.h"
 %}
 
%include "std_string.i"


%include "LinearMath/btScalar.h"
%include "LinearMath/btMinMax.h"

%include "LinearMath/btVector3.h"
%include "LinearMath/btMatrix3x3.h"
%include "LinearMath/btTransform.h"

