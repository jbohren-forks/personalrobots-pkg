///////////////////////////////////////////////////////////////////////////////
// The image_viewer package provides a simple SDL-based image viewer
//
// Copyright (C) 2008  Morgan Quigley
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

#include "ros/ros_slave.h"
#include "SDL/SDL.h"
#include "common_flows/FlowImage.h"
#include "common_flows/ImageCodec.h"

class ImageViewer : public ROS_Slave
{
public:
  FlowImage *image;
  ImageCodec<FlowImage> *codec;
  SDL_Surface *screen, *blit_prep;

  ImageViewer() : ROS_Slave(), blit_prep(NULL)
  {
    register_sink(image = new FlowImage("image"), 
      ROS_CALLBACK(ImageViewer, image_cb));
    codec = new ImageCodec<FlowImage>(image);
    register_with_master();
  }
  virtual ~ImageViewer() { if (blit_prep) SDL_FreeSurface(blit_prep); }
  bool sdl_init()
  {
    screen = SDL_SetVideoMode(320, 240, 24, 0);
    return (screen ? true : false);
  }
  void image_cb()
  {
    if (!screen)
      return; // paranoia. shouldn't happen. we should have bailed by now.
    if (screen->h != image->height || screen->w != image->width)
    {
      printf("resizing for a %d by %d image\n", image->width, image->height);
      screen = SDL_SetVideoMode(image->width, image->height, 24, 0);
      if (!screen)
      {
        fprintf(stderr, "woah! couldn't resize the screen to (%d,%d)\n",
          image->width, image->height);
        exit(1);
      }
    }
    if (!blit_prep || 
         blit_prep->w != image->width || 
         blit_prep->h != image->width)
    {
      if (blit_prep)
        SDL_FreeSurface(blit_prep);
      blit_prep = SDL_CreateRGBSurface(SDL_HWSURFACE | SDL_SRCCOLORKEY, 
        image->width, image->height, 24, 
        0x0000ff, 0x00ff00, 0xff0000, 0);
    }
    
    uint8_t *raster = codec->get_raster(); // decompress if required
    int row_offset = 0;
    for (int row = 0; row < image->height; row++, row_offset += screen->pitch)
      memcpy((char *)blit_prep->pixels + row_offset, raster + (row * image->width * 3), image->width * 3);
    
    if (SDL_MUSTLOCK(screen))
      if (SDL_LockSurface(screen) < 0)
      {
        printf("woah, failed to lock SDL surface.\n");
        return;
      }

    SDL_BlitSurface(blit_prep, NULL, screen, NULL);
/*

    if (screen->pitch == image->width * 3)
      memcpy(screen->pixels, image->raster, image->bytes_per_raster());
    else
    {
      int row_offset = 0;
      for (int row = 0; row < image->height; row++, row_offset += screen->pitch)
        memcpy((char *)screen->pixels + row_offset, image->raster + (row * image->width * 3), image->width * 3);
    }
 */   
    if (SDL_MUSTLOCK(screen))
      SDL_UnlockSurface(screen);
    
    SDL_UpdateRect(screen, 0, 0, screen->w, screen->h);
  }
};

int main(int argc, char **argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "SDL initialization error: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  ImageViewer v;
  if (!v.sdl_init())
  {
    fprintf(stderr, "SDL video startup error: %s\n", SDL_GetError());
    exit(1);
  }
  while (v.happy())
    usleep(1000000);
  return 0;
}

