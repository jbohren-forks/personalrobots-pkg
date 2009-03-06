#include "prosilica/prosilica.h"
#include <cstdio>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s <IP address> <file>\n", argv[0]);
    return 0;
  }

  char buffer[512];
  FILE* file = fopen(argv[2], "rb");
  assert(file);
  fseek(file, 0, SEEK_END);
  size_t size = ftell(file);
  if (size > 512) {
    printf("File is too big!\n");
    return -1;
  }
  fseek(file, 0, SEEK_SET);
  fread(buffer, 1, size, file);
  fclose(file);
  
  prosilica::init();

  // Make sure cam's destructor is called before fini
  {
    prosilica::Camera cam(argv[1]);
    cam.writeUserMemory(buffer, size);
  }

  prosilica::fini();
  
  return 0;
}
