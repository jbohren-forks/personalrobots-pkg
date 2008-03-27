///////////////////////////////////////////////////////////////////////////////
// The sharks package provides a triangulation-based laser scanner.
//
// Copyright (C) 2008, Morgan Quigley, Stanford Univerity
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names 
//     of its contributors may be used to endorse or promote products derived 
//     from this software without specific prior written permission.
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

#include <sstream>
#include "sharks/sharks.h"
#include "axis_cam/axis_cam.h"
#include "ipdcmot/ipdcmot.h"
#include "kbhit.h"

Sharks::Sharks(string axis_ip, string ipdcmot_ip)
  : axis_ip(axis_ip), ipdcmot_ip(ipdcmot_ip),
    cam_ok(true), mot_ok(true)
{
  cam = new AxisCam(axis_ip);
  // make sure we can get an image
  uint8_t *jpeg_buf;
  uint32_t jpeg_buf_size;
  if (!cam->get_jpeg(&jpeg_buf, &jpeg_buf_size))
    printf("woah! couldn't get an image from [%s]\n",
      axis_ip.c_str());
  else
  {
    printf("cam ok. grabbed a %d-byte image.\n",
      jpeg_buf_size);
  }
  printf("entering ipdcmot construct\n");
  mot = new IPDCMOT(ipdcmot_ip, 0, false);
  printf("done with ipdcmot construct\n");
}

Sharks::~Sharks()
{
  printf("sharks destructor\n");
  mot->stop();
}

bool Sharks::get_and_save_image(string filename)
{
  uint8_t *jpeg_buf;
  uint32_t jpeg_buf_size;
  if (!cam->get_jpeg(&jpeg_buf, &jpeg_buf_size))
  {
    printf("woah! no image\n");
    return false;
  }
  //printf("got a %d-byte image\n", jpeg_buf_size);
  FILE *f = fopen(filename.c_str(), "wb");
  fwrite(jpeg_buf, 1, jpeg_buf_size, f);
  fclose(f);
  return true;
}

void Sharks::loneshark()
{
  // this function is a standalone data-collection routine.
  const double left_bound = 90.0, right_bound = 130.0;
  int image_count = 1;
  init_keyboard();
  mot->set_pos_deg_blocking(left_bound);
  mot->set_patrol(left_bound, right_bound, 10, 1);
  printf("press any key to stop scanning\n");
  while (!_kbhit())
  {
    double pos;
    if (!mot->get_pos_blocking(&pos, NULL, 1))
    {
      printf("woah! couldn't get position\n");
      break;
    }
    printf(".");
    fflush(stdout);
    char fnamebuf[500];
    sprintf(fnamebuf, "img_%06d_%012.6f.jpg", image_count, pos);
    image_count++;
    if (!get_and_save_image(fnamebuf))
    {
      printf("woah! couldn't get an image\n");
      break;
    }
  }
  mot->set_pos_deg_blocking(left_bound); // stop the patrol
  char c = _getch();
  printf("you pressed: [%c]\n", c);
  close_keyboard();
}

