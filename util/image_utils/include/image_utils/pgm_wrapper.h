#ifndef IMAGE_UTILS_PGM_WRAPPER_H
#define IMAGE_UTILS_PGM_WRAPPER_H

// this guy encapsulates reading / writing 8-bit pgm images
#include <string>
#include <cstdio>
#include <cassert>
using std::string;

class PgmWrapper
{
public:
  PgmWrapper();
  ~PgmWrapper();

  static char *write_file(string filename, int width, int height, 
                          uint8_t *raster)
  {
    FILE *f = fopen(filename.c_str(), "wb");
    if (!f)
      return "couldn't open file to write pgm";
    fprintf(f, "P5\n%d %d\n255\n", width, height);
    fwrite(raster, 1, width*height, f);
    fclose(f);
    return NULL; // no error
  }
};

#endif

