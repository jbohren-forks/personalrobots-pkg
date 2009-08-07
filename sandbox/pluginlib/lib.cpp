#include <stdio.h>

typedef int (*funtype)(void);

funtype table[32];

int register_controller(int i, funtype j)
{
  table[i] = j;
  return i;
}

#define ADD_CONTROLLER(i,j) \
        const int SOMETHING_##i = register_controller(i,j)

int foo(void)
{
  puts("foo");
  return 0;
}
int bar(void)
{
  puts("bang");
  return 1;
}
int bat(void)
{
  puts("bat");
  return 2;
}

ADD_CONTROLLER(0, foo);
ADD_CONTROLLER(1, bar);
ADD_CONTROLLER(2, bat);

