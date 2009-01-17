/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2008,  Morgan Quigley
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

#include "ros/node.h"
#include "SDL/SDL.h"
#include "std_msgs/Image.h"
#include "image_utils/image_codec.h"
#include "simple_sdl_gui/SDLKeyEvent.h"

class Simple_SDL_GUI : public ros::Node
{
public:
  std_msgs::Image image;
  ImageCodec<std_msgs::Image> codec;
  
  simple_sdl_gui::SDLKeyEvent key;

  SDL_Surface *screen, *blit_prep;
  SDL_Event event;

  bool new_image;

  Simple_SDL_GUI() : ros::Node("simple_sdl_gui"), codec(&image), blit_prep(NULL)
  {
    subscribe("image", image, &Simple_SDL_GUI::image_received);
    advertise<simple_sdl_gui::SDLKeyEvent>("key");

    new_image = false;
  }

  virtual ~Simple_SDL_GUI() { if (blit_prep) SDL_FreeSurface(blit_prep); }

  bool sdl_init()
  {
    screen = SDL_SetVideoMode(320, 240, 24, 0);
    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);//People expect Key Repeat
    return (screen ? true : false);
  }

  void refresh() {
    check_keyboard();
    display_image();
  }

  void check_keyboard() {
    while(SDL_PollEvent(&event)) {
      if ((event.type == SDL_KEYDOWN) || (event.type == SDL_KEYUP))
	{
	  key.state = event.key.state;
	  key.sym = event.key.keysym.sym;
	  key.mod = event.key.keysym.mod;
	  publish("key", key);
      }
    }
  }

  void image_received() {
    new_image = true;
  }
 
  void display_image()
  {
    if (!screen)
      return; // paranoia. shouldn't happen. we should have bailed by now.

   //Only show the image if a new one has arrived
    if (new_image) {
      uint8_t *raster = codec.inflate();
      
      //There is some thread unsafeness here if image changes size
      if (screen->h != image.height || screen->w != image.width)
	{
	  screen = SDL_SetVideoMode(image.width, image.height, 24, 0);
	  if (!screen)
	    {
	      fprintf(stderr, "woah! couldn't resize the screen to (%d,%d)\n",
		      image.width, image.height);
	      exit(1);
	    }
	}

      if (!blit_prep || blit_prep->w != image.width || blit_prep->h != image.width)
	{
	  if (blit_prep)
	    SDL_FreeSurface(blit_prep);
	  blit_prep = SDL_CreateRGBSurface(SDL_HWSURFACE | SDL_SRCCOLORKEY, 
					   image.width, image.height, 24, 
					   0x0000ff, 0x00ff00, 0xff0000, 0);
	}

      // NOTE: get_raster internally locks the image mutex
      //
      int row_offset = 0;
      for (int row = 0; row < image.height; row++, row_offset += screen->pitch)
	memcpy((char *)blit_prep->pixels + row_offset, raster + (row * image.width * 3), image.width * 3);
      
      if (SDL_MUSTLOCK(screen))
	if (SDL_LockSurface(screen) < 0)
	  {
	    printf("woah, failed to lock SDL surface.\n");
	    return;
	  }
      
      SDL_BlitSurface(blit_prep, NULL, screen, NULL);
      
      if (SDL_MUSTLOCK(screen))
	SDL_UnlockSurface(screen);
      
      SDL_UpdateRect(screen, 0, 0, screen->w, screen->h);
    }

    new_image = false;

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "SDL initialization error: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  Simple_SDL_GUI s;

  if (!s.sdl_init())
  {
    fprintf(stderr, "SDL video startup error: %s\n", SDL_GetError());
    exit(1);
  }

  while (s.ok())
  {
    s.refresh();
    usleep(1000);
  }
  return 0;

  ros::fini();
}

