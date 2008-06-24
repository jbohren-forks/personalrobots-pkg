#ifndef IMAGE_UTILS_PPM_WRAPPER_H
#define IMAGE_UTILS_PPM_WRAPPER_H

// this guy encapsulates reading / writing ppm images
#include <string>
#include <cstdio>
#include <cassert>
using std::string;

class PpmWrapper
{
public:
  PpmWrapper();
  ~PpmWrapper();

  static const char *write_file(string filename, int width, int height, 
                                string colorspace, uint8_t *raster)
  {
    if (colorspace != string("rgb24") && colorspace != string("bgr24"))
      return "woah! PpmWrapper can only handle rgb24 or bgr24 images";
    FILE *f = fopen(filename.c_str(), "wb");
    if (!f)
      return "couldn't open file to write ppm";
    fprintf(f, "P6%d %d\n255\n", width, height);
    if (colorspace == string("rgb24"))
      fwrite(raster, 1, width*height*3, f);
    else if (colorspace == string("bgr24"))
    {
      uint8_t *bgr = new uint8_t[width*height*3];
      for (int y = 0; y < height; y++)
        for(int x = 0; x < height; x++)
        {
          uint8_t *p = raster + y * width * 3 + x * 3;
          uint8_t *q = bgr    + y * width * 3 + x * 3;
          q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
        }
      fwrite(bgr, 1, width * height * 3, f);
      delete[] bgr;
    }
    else
      assert(0);
    fclose(f);
    return NULL; // no error
  }
};

#endif

