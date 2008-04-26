///////////////////////////////////////////////////////////////////////////////
// The sharks package provides a triangulation-based laser scanner.
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

#ifndef SHARKS_SHARKS_H
#define SHARKS_SHARKS_H

#include <string>
#include "axis_cam/axis_cam.h"
#include "ipdcmot/ipdcmot.h"
#include "image_utils/jpeg_wrapper.h"
#include "SDL/SDL.h"
using namespace std;

class Sharks
{
public:
  Sharks(string axis_ip, string ipdcmot_ip, bool gui = false);
  ~Sharks();
  void calibrate();
  void loneshark();
  inline bool ok() { return cam_ok && mot_ok; }
  // gui functions (only do anything if gui=true in the constructor)
  void pump_gui_event_loop();
  void gui_spin();

  class img_diff_t
  {
    public:
      int r, g, b;
      img_diff_t() : r(0), g(0), b(0) { }
  };
  img_diff_t image_diff(int w, int h, uint8_t *img1, uint8_t *img2, int thresh = -100000);
  bool load_config_file(string filename);
  void manual_calibration();

private:
  bool cam_ok, mot_ok, gui;
  string axis_ip, ipdcmot_ip;
  AxisCam *cam;
  IPDCMOT *mot;
  bool get_and_save_image(string filename);
  SDL_Surface *screen, *blit_prep;
  double left_laser_bound, right_laser_bound;
  JpegWrapper *jpeg_wrapper;
  void display_image(int width, int height, uint8_t *raster);
  void render_image();
  double left_scan_extent, right_scan_extent;
  void handle_key(int k);
  bool manual_calibration_complete;
  int man_iris, man_focus;
};

#endif

