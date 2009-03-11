/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*#########################################
 * ostpng.cpp
 *
 * OST (Open STereo) PNG read/write functions
 * Uses LodePNG source
 *
 *#########################################
 */

/**
 ** ostpng.cpp
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **/


#include <stdlib.h>
#include <malloc.h>
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <iostream>
#ifdef WIN32
#pragma warning (disable: 4267 4244 4800 4996)
#include <time.h>
#else
#include <sys/time.h>
#endif
#include "dcam/stereodcam.h"
#include "lodepng.h"

using namespace std;

// alignment on allocation, for image buffers
#define MEMALIGN(x) memalign(16,x)
#define MEMFREE(x) {if (x) free(x);}

//
// Load a PNG file into a grayscale buffer
// Main helper fn for grayscale, ONLY reads grayscale images
// Creates storage for image, should be freed later
//

uint8_t *
load_png_grayscale(char *fname, color_coding_t *cc, int *width, int *height, int *size)
{
  if (cc) *cc = COLOR_CODING_NONE; // just in case

  // read in file, if it exists
  std::vector<unsigned char> buffer, image;
  LodePNG::loadFile(buffer, fname); // load the image
  LodePNG::Decoder decoder;
  // figure out the type of file
  decoder.inspect(buffer.empty() ? 0 : &buffer[0], (unsigned)buffer.size());
  // check errors
  if (decoder.hasError())
    {
      std::cout << "[OST] error: " << decoder.getError() << std::endl;
      return NULL;
    }

  // check type
  if (!decoder.isGreyscaleType())
    {
      std::cout << "[OST] error: not a greyscale image" << endl;
      return NULL;
    }

  // no color conversion
  LodePNG_DecodeSettings st = decoder.getSettings();
  st.color_convert = 0;
  decoder.setSettings(st);


  // decode the PNG
  decoder.decode(image, buffer.empty() ? 0 : &buffer[0], (unsigned)buffer.size());
  
  // image size
  int w = decoder.getWidth();
  int h = decoder.getHeight();
  int bpp = decoder.getBpp();
  printf("w: %d  h: %d  bpp: %d\n", w, h, bpp);
  int bypp = (bpp+7)/8;

  // check errors
  if (decoder.hasError())
    {
      std::cout << "[OST] error: " << decoder.getError() << std::endl;
      return NULL;
    }

  // check color and convert to grayscale if necessary

  if (width) *width = w;
  if (height) *height = h;
  if (size) *size = w*h*bypp;	// should be the same as image vector size

  if (cc)
    {
      switch (bypp)
	{
	case 1:			// one byte per pixel
	  *cc = COLOR_CODING_MONO8;
	  break;
	case 2:			// two bytes per pixel
	  *cc = COLOR_CODING_MONO16;
	  break;
	default:
	  *cc = COLOR_CODING_MONO8;
	  break;
	}
    }

  // copy to array
  int ss = image.size();
  printf("Size: %d\n", ss);
  uint8_t *buf = (uint8_t *)MEMALIGN(ss);
  for (int i=0; i<ss; i++) buf[i] = image[i];

  return buf;
}
