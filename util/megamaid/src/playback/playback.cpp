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

class Playback : public ros::node
{
public:
  LogPlayer<> *bags;
  size_t num_bags;
  Playback(vector<string> bag_names) : node("playback"), num_bags(0)
  { 
    num_bags = bag_names.size();
    bags = new LogPlayer<>[num_bags];
    ros::Time t = ros::Time::now();
    for (size_t i = 0; i < num_bags; i++)
    {
      printf("%s\n", bag_names[i].c_str());
      if (!bags[i].open_log(bag_names[i], t))
        throw std::runtime_error("couldn't open bag file\n");
    }
    // all the bags opened up OK. advertise them.
    for (size_t i = 0; i < num_bags; i++)
      advertise(bags[i].topic_name, bags[i]);
  }
  virtual ~Playback()
  {
    delete[] bags;
    bags = NULL;
  }
  void play()
  {
    while (ok())
    {
      bool keep_going = false;
      ros::Time min_t(-1,0);
      ros::Time soon = ros::Time::now() + ros::Duration(0,5000);

      vector<size_t> inds;

      for (size_t i = 0; i < num_bags; i++)
      {
        if (!bags[i].done)
        {
          keep_going = true;
          ros::Time next = bags[i].time_of_msg();
          if (next < soon)
          {
            min_t = next;
            inds.push_back(i);
          } else if (next < min_t)
          {
            min_t = next;
          }
        }
      }
      if (!keep_going)
        break;

      ros::Duration delta = min_t - soon;

      if (delta > ros::Duration(0,5000))
        usleep(delta.to_ll()/1000 - 5);

      for (vector<size_t>::iterator i = inds.begin(); i != inds.end(); i++)
      {
        publish(bags[*i].topic_name, bags[*i]);
        if (!bags[*i].read_next_msg())
          bags[*i].done = true;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc <= 1)
  {
    printf("\nusage: playback BAG1 [BAG2] ...\n  this is a good place to use "
           "the shell * feature\n\n");
    return 1;
  }
  vector<string> bag_names;
  for (int i = 1; i < argc; i++)
    bag_names.push_back(argv[i]);
  Playback pb(bag_names);
  pb.play();
  ros::fini();
  return 0;
}
