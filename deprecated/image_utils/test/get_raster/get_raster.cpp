///////////////////////////////////////////////////////////////////////////////
// The image_flows package provides image transport, codec wrapping, and 
// various handy image utilities.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "common_flows/FlowImage.h"
#include "common_flows/ImageCodec.h"

void toast(char *msg)
{
  int serr = errno;
  printf("%s: (%d) %s\n", msg, serr, strerror(serr));
  exit(1);
}

int main(int argc, char **argv)
{
  struct stat sbuf;
  const char *fname = "test.jpg";
  FlowImage image("image");
  if (stat(fname, &sbuf))
    toast("stat error");
  printf("file has %d bytes.\n", sbuf.st_size);
  if (sbuf.st_size <= 0)
    toast("file has zero size.\n");
  image.set_data_size(sbuf.st_size);
  FILE *f = fopen(fname, "rb");
  int num_read = fread(image.data, 1, sbuf.st_size, f);
  if (num_read != sbuf.st_size)
    toast("couldn't read entire file\n");
  printf("read %d bytes\n", num_read);
  image.compression = "jpeg";
  image.colorspace = "rgb24";
  image.width = 200;
  image.height = 153;
  ImageCodec<FlowImage> codec(&image);
  uint8_t *raster;
  if (!(raster = codec.get_raster()))
    printf("couldn't get raster\n");
  else
  {
    printf("got raster of %d bytes\n", codec.get_raster_size());
    codec.write_file("out.ppm");
    codec.write_file("out.jpg", 5);
  }

  return 0;
}

