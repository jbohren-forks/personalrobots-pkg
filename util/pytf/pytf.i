 %module pytf
 %{
 /* Includes the header in the wrapper code */
 #include "pytf.h"
 %}
 
%include "std_string.i"

 /* Parse the header file to generate wrappers */
%include "pytf.h"
