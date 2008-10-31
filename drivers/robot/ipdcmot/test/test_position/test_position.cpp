///////////////////////////////////////////////////////////////////////////////
// The ipdcmot package provides a library that talks to the FMOD IP-based 
// motor controller. I just have their single-channel 1.5A box, but perhaps
// some of this code will be useful on their other boxes as well.
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
#include <cstdlib>
#include "ipdcmot/ipdcmot.h"

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: test_position POS\nwhere POS is in degrees from home\n");
    return 0;
  }
  IPDCMOT *mot = new IPDCMOT("192.168.1.38", 0, false);
  double pos = 30;
  pos = atof(argv[1]);
  mot->set_pos_deg_blocking(pos);

  //printf("sleeping...\n");
  //usleep(1000000);
  //printf("going to 20 degrees\n");
  //mot->set_pos_deg_blocking(20);
  //printf("done. going to 40 degrees\n");
  //mot->set_pos_deg_blocking(40);
  //printf("done.\n");


  delete mot;
  return 0;
}

