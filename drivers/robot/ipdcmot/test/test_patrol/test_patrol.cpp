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

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include "ipdcmot/ipdcmot.h"

IPDCMOT *mot = NULL;

void safe_term(int)
{
  printf("stopping motors before terminating...\n");
  if (mot)
    mot->stop();
  exit(0);
}

int main(int argc, char **argv)
{
  signal(SIGTERM, safe_term);
  signal(SIGINT, safe_term);
  signal(SIGQUIT, safe_term);
  signal(SIGHUP, safe_term);
  mot = new IPDCMOT("192.168.1.38", 0);
  printf("press enter to patrol\n");
  fgetc(stdin);
  mot->set_patrol(10, 50, 1, 1);
  printf("press enter to quit\n");
//  fgetc(stdin);
  while(1)
  {
    usleep(10000);
    double pos_deg;
    mot->get_pos_blocking(&pos_deg, NULL, 1);
    pos_deg *= 3.1415926/180.0;
    printf("%f\n", pos_deg);
  }
  printf("good night.\n");
  delete mot;
  return 0;
}

