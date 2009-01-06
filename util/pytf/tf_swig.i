 %module tf_swig
 %{
 /* Includes the header in the wrapper code */
 #include "tf/tf.h"
 #include "tf/time_cache.h"
 #include "tf/transform_datatypes.h"
 #include "LinearMath/btTransform.h"
 #include <vector>
 %}
 

 /* Parse the header file to generate wrappers */
 %include "tf/tf.h"


