#include <dlfcn.h>
#include <assert.h>
#include <unistd.h>
#include <stdio.h>

typedef int (*funtype)(void);

void foo(void);

int
main(void)
{
  void* handle = NULL;
  //funtype fun;
  funtype* table;
  int i;

  for(;;)
  {
    if(handle)
    {
      puts("closing");
      assert(dlclose(handle) == 0);
      handle = NULL;
    }

    puts("opening");
    handle = dlopen("./lib/libfoo.so", RTLD_NOW);
    assert(handle);

    table = (funtype*)dlsym(handle, "table");
    printf("dlsym: %s\n", dlerror());
    assert(table);

    //puts("calling");
    //(fun)();

    for(i=0;i<4;i++)
    {
      if(table[i])
        printf("%d: %d\n", i, table[i]());
    }

    puts("sleeping for 10 seconds");
    usleep(10000000);
  }

  return 0;
}
