#ifndef _BUILD_H_
#define _BUILD_H_


//#define DEVICE_BUILD
#define LIBRARY_BUILD


// Altium compiler
#ifdef __TASKING__
# define PACKED_ATTRIBUTE __packed__
# define debug(fmt,args...)
#endif

// GCC
#ifdef __GNUC__
# define PACKED_ATTRIBUTE __attribute__((__packed__))
# define __at(var)
# define __interrupt(var)
# define __mfc0(var) 0
# define __mtc0(var1, var2)
# define __nop()
# define debug(...) printf(__VA_ARGS__)
//# define debug(...)
# define __asm(var)

#include <stdio.h>
#endif





#endif //BUILD_H
