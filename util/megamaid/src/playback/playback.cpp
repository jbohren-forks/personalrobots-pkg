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

#include "logging/LogPlayer.h"

using namespace std;

void doPublish(string name, ros::msg* m, ros::Time t, void* n)
{
  ros::Time now = ros::Time::now();

  ros::Duration delta = t - ros::Time::now();

  if (delta > ros::Duration(0, 5000))
    usleep(delta.to_ll()/1000 - 5);
  
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

  vector<LogPlayer*> players;

  ros::Time start = ros::Time::now();

  for (int i = 1; i < argc; i++)
  {
    LogPlayer* l = new LogPlayer;

    if (l->open(argv[i], start)) {

      std::vector<std::string> names = l->getNames();

      for (std::vector<std::string>::iterator i = names.begin(); i != names.end(); i++)
      {
        n.advertise<AnyMsg>(*i);
      }

      l->addHandler<AnyMsg>(string("*"), &doPublish, (void*)(&n), false);
      
      players.push_back(l);

    } else {
      n.shutdown();
      break;
    }
  }

  bool done = false;;
  while(n.ok() && !done)
  {
    done = true;
    LogPlayer* next_player = 0;
    ros::Time min_t = ros::Time(-1); // This should be the maximum unsigned int;
    for (vector<LogPlayer*>::iterator player_it = players.begin();
         player_it != players.end();
         player_it++)
    {
      if ((*player_it)->isDone())
      {
        continue;
      }
      else
      {
        done = false;
        ros::Time t = (*player_it)->get_next_msg_time();
        if (t < min_t)
        {
          next_player = (*player_it);
          min_t = (*player_it)->get_next_msg_time();
        }
      }
    }
    if (next_player)
      next_player->nextMsg();
  }

  ros::fini();
  return 0;
}
