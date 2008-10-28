///////////////////////////////////////////////////////////////////////////////
// The megamaid package provides logging and playback
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
//////////////////////////////////////////////////////////////////////////////

#include <time.h>
#include <sys/stat.h>
#include "ros/node.h"
#include "ros/time.h"
#include <string>
#include <cstdio>
#include <limits>

#include "logging/LogPlayer.h"

using namespace std;

void doPublish(string name, ros::msg* m, ros::Time t, void* n)
{
  if (((ros::node*)(n))->advertise<AnyMsg>(name, 0))
    usleep(200000);

  std::cout << "Hit enter to continue.";
  std::cout.flush();
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  
  ((ros::node*)(n))->publish(name, *m);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc <= 1)
  {
    printf("\nusage: playback BAG1 [BAG2] ...\n  this is a good place to use "
           "the shell * feature\n\n");
    return 1;
  }

  ros::node n("player");  // Ros peer style usage.

  MultiLogPlayer player;

  std::vector<std::string> ags;
  for (int i = 1; i < argc; i++)
    ags.push_back(argv[i]);

  ros::Time start = ros::Time::now();

  if (player.open(ags, start))
  {
    player.addHandler<AnyMsg>(string("*"), &doPublish, (void*)(&n), false);
  }

  while(n.ok())
  {
    if (!player.nextMsg())
    {
      n.self_destruct();
    }
  }

  usleep(100000);
  ros::fini();
  return 0;
}
