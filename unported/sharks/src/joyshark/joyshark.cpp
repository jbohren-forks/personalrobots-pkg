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

#include <cstdio>
#include "sharks/sharks.h"
#include "joy/MsgJoy.h"
#include "ros/node.h"

using namespace ros;

class JoyShark : public Node
{
public:
  MsgJoy joy;
  int prev_buttons;
  Sharks *sharks;

  JoyShark() : Node("joyshark"), prev_buttons(0)
  {
    sharks = new Sharks("192.168.1.90", "192.168.1.38", true);
    subscribe("joy", joy, &JoyShark::joy_cb);
  }
  void joy_cb()
  {
    if ((joy.buttons & 0x1) && !(prev_buttons & 0x1))
    {
      sharks->load_config_file("config.txt");
      sharks->loneshark();
    }
    prev_buttons = joy.buttons;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  JoyShark shark;
  shark.spin();
  return 0;
}
/*
  printf("construct Sharks object...\n");
  Sharks *sharks = new Sharks("192.168.1.90", "192.168.1.38", true);

  //printf("press enter to scan or type 'a' then enter to abort\n");
  //char c = fgetc(stdin);
  
  if (argc == 1)
    sharks->load_config_file("config.txt");
  else if (argc == 2 && argv[1] == string("--calibrate"))
  { 
    sharks->manual_calibration();
    delete sharks; return 0;
  }

//    sharks->calibrate();
//  else if (argc == 2 && argv[1] == "--manual")
  else
    sharks->load_config_file(argv[1]);
  sharks->loneshark();

  //sharks->gui_spin();

  delete sharks;
  return 0;
}
*/

