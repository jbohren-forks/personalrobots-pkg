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
#include <cstdio>
#include "sharks/sharks.h"
#include "axis_cam/axis_cam.h"
#include "ipdcmot/ipdcmot.h"
#include "kbhit.h"

Sharks::Sharks(string axis_ip, string ipdcmot_ip, bool gui)
  : axis_ip(axis_ip), ipdcmot_ip(ipdcmot_ip),
    cam_ok(true), mot_ok(true), gui(gui),
    screen(NULL), blit_prep(NULL),
    left_laser_bound(70.0), right_laser_bound(190.0),
    left_scan_extent(-100000), right_scan_extent(-100000)
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
  jpeg_wrapper = new JpegWrapper();
  if (gui)
  {
    SDL_Init(SDL_INIT_VIDEO);
    screen = SDL_SetVideoMode(704, 480, 24, 0);
  }
}

Sharks::~Sharks()
{
  printf("sharks destructor\n");
  mot->stop();
  if (blit_prep)
    SDL_FreeSurface(blit_prep);
  if (gui)
    SDL_Quit();
}

void Sharks::render_image()
{
  if (!gui || !screen || !blit_prep)
    return;
  if (SDL_MUSTLOCK(screen))
    SDL_LockSurface(screen);
  SDL_BlitSurface(blit_prep, NULL, screen, NULL);
  if (SDL_MUSTLOCK(screen))
    SDL_UnlockSurface(screen);
  SDL_UpdateRect(screen, 0, 0, screen->w, screen->h);
}
  
void Sharks::display_image(int width, int height, uint8_t *raster)
{
  if (!gui || !screen)
    return;
  if (!blit_prep ||
       blit_prep->w != width ||
       blit_prep->h != height)
  {
    if (blit_prep)
      SDL_FreeSurface(blit_prep);
    blit_prep = SDL_CreateRGBSurface(SDL_HWSURFACE | SDL_SRCCOLORKEY,
      width, height, 24,
      0x0000ff, 0x00ff00, 0xff0000, 0);
  }
  int row_offset = 0;
  for (int row = 0; row < height; row++, row_offset += screen->pitch)
    memcpy((char *)blit_prep->pixels + row_offset, 
           raster + (row * width * 3), width * 3);
  render_image();
}

Sharks::img_diff_t Sharks::image_diff(int width, int height, uint8_t *img1, uint8_t *img2, int thresh)
{
  img_diff_t diff;
  for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++)
    {
      uint8_t *p1 = img1 + y*width*3 + x*3;
      uint8_t *p2 = img2 + y*width*3 + x*3;
      int r1 = *p1, g1 = *(p1+1), b1 = *(p1+2);
      int r2 = *p2, g2 = *(p2+1), b2 = *(p2+2);
      if (r1 - r2 > thresh)
        diff.r += r1 - r2;
      if (g1 - g2 > thresh)
        diff.g += g1 - g2;
      if (b1 - b2 > thresh)
        diff.b += b1 - b2;
    }
  return diff;
}

void Sharks::calibrate()
{
  init_keyboard();
  printf("setting camera to autofocus\n");
  cam->set_focus(0);
  printf("letting camera autofocus for 1 second\n");
  usleep(1000000);
  printf("fixing focus and iris to current values\n");
  cam->set_focus(0, true);
  cam->set_iris(0, true);
  mot->set_pos_deg_blocking(left_laser_bound); // get it way out of the way
  uint8_t *jpeg_buf;
  uint32_t jpeg_buf_size;
  cam->get_jpeg(&jpeg_buf, &jpeg_buf_size);
  printf("jpeg_buf_size = %d first bytes = %d %d\n", jpeg_buf_size, jpeg_buf[0], jpeg_buf[1]);
  jpeg_wrapper->decompress_jpeg_buf((char *)jpeg_buf, jpeg_buf_size);
  int width = jpeg_wrapper->width(), height = jpeg_wrapper->height();
  int rs = jpeg_wrapper->raster_size();
  display_image(width, height, jpeg_wrapper->get_raster());
  uint8_t *nolaser_buf = new uint8_t[rs];
  uint8_t *diff_image = new uint8_t[rs];
  memcpy(nolaser_buf, jpeg_wrapper->get_raster(), rs);
  int max_red_diff = 0;
  double max_red_diff_ang = 135;
  /*
  for (double ang = left_laser_bound; ang <= right_laser_bound; ang += 5.0)
  {
    mot->set_pos_deg_blocking(ang);
    cam->get_jpeg(&jpeg_buf, &jpeg_buf_size);
    jpeg_wrapper->decompress_jpeg_buf((char *)jpeg_buf, jpeg_buf_size);
    img_diff_t diff = image_diff(jpeg_wrapper->width(), jpeg_wrapper->height(), 
                                 jpeg_wrapper->get_raster(), nolaser_buf, 20);
    printf("ang = %f  delta = (%d, %d, %d)\n", ang, diff.r, diff.g, diff.b);
    if (diff.r > max_red_diff)
    {
      max_red_diff = diff.r;
      max_red_diff_ang = ang;
    }
    for (int y = 0; y < height; y++)
      for (int x = 0; x < width; x++)
      {
        uint8_t *p1 = jpeg_wrapper->get_raster() + y*width*3 + x*3;
        uint8_t *p2 = nolaser_buf + y*width*3 + x*3;
        uint8_t *p3 = diff_image + y*width*3 + x*3;
        int      r1 = *(p1+0), g1 = *(p1+1), b1 = *(p1+2);
        int      r2 = *(p2+0), g2 = *(p2+1), b2 = *(p2+2);
        uint8_t *r3 =   p3+0, *g3 =   p3+1, *b3 =   p3+2;

        if (r1 - r2 > 0)
          *r3 = (uint8_t)(r1 - r2);
        else
          *r3 = 0;
        
        if (g1 - g2 > 0)
          *g3 = (uint8_t)(g1 - g2);
        else
          *g3 = 0;

        if (b1 - b2 > 0)
          *b3 = (uint8_t)(b1 - b2);
        else
          *b3 = 0;
        
        //*r3 = (uint8_t)r1;
        //*g3 = 0;
        //*b3 = 0;
      }
    display_image(width, height, diff_image); //jpeg_wrapper->get_raster());
  }
  */
  mot->set_pos_deg_blocking(max_red_diff_ang);
  printf("setting camera to autofocus again\n");
  cam->set_focus(0);
  printf("letting camera autofocus for 3 seconds\n");
  usleep(3000000);
  printf("fixing focus and iris to current values\n");
  cam->set_focus(0, true);
  
  int red[256];
  // do a binary search on the iris
  for (int diris = 1000; diris > 50; diris /= 2)
  {
    int direction;
    for (int attempt = 0; attempt < 10; attempt++)
    {
      if (!cam->get_jpeg(&jpeg_buf, &jpeg_buf_size))
      {
        printf("woah! couldn't get an image\n");
        continue;
      }
      printf("jpeg buf size = %d\n", jpeg_buf_size);
      jpeg_wrapper->decompress_jpeg_buf((char *)jpeg_buf, jpeg_buf_size);
      display_image(width, height, jpeg_wrapper->get_raster());
      for (int i = 0; i < 256; i++)
        red[i] = 0;
      for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
          uint8_t *p1 = jpeg_wrapper->get_raster() + y*width*3 + x*3;
          red[*p1]++;
        }
      printf("red-saturated pixels: %d\n", red[255]);
      const int tgt_sat = 500;
      if (attempt == 0)
        direction = (red[255] > tgt_sat ? -1 : 1);
      if (direction == -1)
      {
        // see if we've closed the iris enough to not saturate big swaths of the image
        if (red[255] <= tgt_sat)
          break; // done!
        cam->set_iris(-diris, true); // else, close the iris some more
      }
      else // direction is -1
      {
        // see if we've opened the iris enough to saturate a bit of the image
        if (red[255] > tgt_sat)
          break;
        cam->set_iris(diris, true);
      }
      if (diris >= 500)
        usleep(500000); // big motions take a while
      printf("direction = %d diris = %d cam iris = %d\n", direction, diris, cam->get_iris());
    }
  }

  // find extents of the laser in the image
  mot->set_pos_deg_blocking(left_laser_bound);
  cam->get_jpeg(&jpeg_buf, &jpeg_buf_size);
  printf("jpeg_buf_size = %d first bytes = %d %d\n", jpeg_buf_size, jpeg_buf[0], jpeg_buf[1]);
  jpeg_wrapper->decompress_jpeg_buf((char *)jpeg_buf, jpeg_buf_size);
  memcpy(nolaser_buf, jpeg_wrapper->get_raster(), rs);
  display_image(width, height, jpeg_wrapper->get_raster());

  const double extent_delta_ang = 2.0;
  double left_image_start = -100000, right_image_start = right_laser_bound;
  for (double ang = left_laser_bound; ang <= right_laser_bound; ang += extent_delta_ang)
  {
    mot->set_pos_deg_blocking(ang);
    cam->get_jpeg(&jpeg_buf, &jpeg_buf_size);
    jpeg_wrapper->decompress_jpeg_buf((char *)jpeg_buf, jpeg_buf_size);
    img_diff_t diff = image_diff(jpeg_wrapper->width(), jpeg_wrapper->height(), 
                                 jpeg_wrapper->get_raster(), nolaser_buf, 20);
    printf("ang = %f  delta = (%d, %d, %d)\n", ang, diff.r, diff.g, diff.b);
    if (diff.r > 10000)
    {
      if (left_image_start < -1000)
      {
        left_image_start = ang - extent_delta_ang;
        printf("found left image start: %f\n", left_image_start);
      }
      right_image_start = ang + extent_delta_ang;
    }

    /*
    if (diff.r > max_red_diff)
    {
      max_red_diff = diff.r;
      max_red_diff_ang = ang;
    }
    */
    for (int y = 0; y < height; y++)
      for (int x = 0; x < width; x++)
      {
        uint8_t *p1 = jpeg_wrapper->get_raster() + y*width*3 + x*3;
        uint8_t *p2 = nolaser_buf + y*width*3 + x*3;
        uint8_t *p3 = diff_image + y*width*3 + x*3;
        int      r1 = *(p1+0), g1 = *(p1+1), b1 = *(p1+2);
        int      r2 = *(p2+0), g2 = *(p2+1), b2 = *(p2+2);
        uint8_t *r3 =   p3+0, *g3 =   p3+1, *b3 =   p3+2;

        if (r1 - r2 > 0)
          *r3 = (uint8_t)(r1 - r2);
        else
          *r3 = 0;
        
        if (g1 - g2 > 0)
          *g3 = (uint8_t)(g1 - g2);
        else
          *g3 = 0;

        if (b1 - b2 > 0)
          *b3 = (uint8_t)(b1 - b2);
        else
          *b3 = 0;
        
        //*r3 = (uint8_t)r1;
        //*g3 = 0;
        //*b3 = 0;
      }
    display_image(width, height, diff_image); //jpeg_wrapper->get_raster());
  }

  printf("scan extents: [%f, %f]\n", left_image_start, right_image_start);
  left_scan_extent = left_image_start;
  right_scan_extent = right_image_start;

  close_keyboard();
  delete[] nolaser_buf;
  delete[] diff_image;
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
  if (left_scan_extent < -500 || right_scan_extent < -500)
  {
    printf("hey! i don't appear to be calibrated.\n");
    return;
  }
  int image_count = 1;
  init_keyboard();
  mot->set_pos_deg_blocking(left_scan_extent);
  mot->set_patrol(left_scan_extent, right_scan_extent, 1, 1);
  printf("press any key to stop scanning\n");
  double prev_mot = -1000;
  while (!_kbhit())
  {
    double pos;
    if (!mot->get_pos_blocking(&pos, NULL, 1))
    {
      printf("woah! couldn't get position\n");
      break;
    }
    if (pos < prev_mot)
    {
      printf("direction change detected. scan complete.\n");
      break;
    }
    prev_mot = pos;
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
  mot->set_pos_deg_blocking(left_laser_bound); // stop the patrol
  char c = _getch();
  printf("you pressed: [%c]\n", c);
  close_keyboard();
}

void Sharks::pump_gui_event_loop()
{
  if (!gui)
    return;
  SDL_Event event;
  while (SDL_PollEvent(&event))
  {
    switch(event.type)
    {
      case SDL_VIDEOEXPOSE: render_image(); break;
    }
  }
}

void Sharks::gui_spin()
{
  if (!gui)
    return;
  init_keyboard();
  while (!_kbhit())
  {
    usleep(1000);
    pump_gui_event_loop();
  }
  close_keyboard();
}

