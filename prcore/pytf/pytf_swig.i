 %module pytf_swig
 %{
 /* Includes the header in the wrapper code */
 #include "pytf_swig.h"
 %}
 
%include "std_string.i"

 /* Parse the header file to generate wrappers */
%include "pytf_swig.h"
