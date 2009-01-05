 %module tf_swig
 %{
 /* Includes the header in the wrapper code */
 #include "../tf/include/tf/tf.h"
 %}
 
 /* Parse the header file to generate wrappers */
 %include "../tf/include/tf/tf.h"