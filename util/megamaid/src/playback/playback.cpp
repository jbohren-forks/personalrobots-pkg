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
using namespace ros;
using namespace std;

ros::Time start;

class Bag : public msg
{
public:
  FILE *log;
  ros::Time playback_start;
  ros::Duration next_msg_dur;
  string topic_name;
  uint8_t *next_msg;
  uint32_t next_msg_size, next_msg_alloc_size;
  bool done;
  Bag() : msg(), log(NULL), next_msg(NULL), next_msg_alloc_size(0), 
          done(false) { }
  virtual ~Bag() { fclose(log); if (next_msg) delete[] next_msg; }
  bool open_log(const string &bag_name, ros::Time _start)
  {
    playback_start = _start;
    log = fopen(bag_name.c_str(), "r");
    if (!log)
    {
      done = true;
      return false;
    }
    char topic_cstr[4096];
    fgets(topic_cstr, 4096, log);
    topic_cstr[strlen(topic_cstr)-1] = 0;
    topic_name = string(topic_cstr);
    printf("topic: [%s]\n", topic_cstr);
    if (!read_next_msg())
      return false;
    return true;
  }
  int32_t usecs_to_next_msg()
  {
    if (done)
      assert(0); // bad bad bad
    ros::Duration cur_log_dur = ros::Time::now() - playback_start;
    ros::Duration delta = next_msg_dur - cur_log_dur;
    return delta.sec * 1000000 + delta.nsec / 1000;
  }
  bool read_next_msg()
  {
    if (!log)
    {
      done = true;
      return false;
    }
    fread(&next_msg_dur.sec, 4, 1, log);
    fread(&next_msg_dur.nsec, 4, 1, log);
    fread(&next_msg_size, 4, 1, log);
    if (feof(log))
    {
      done = true;
      return false;
    }
    if (next_msg_size > next_msg_alloc_size)
    {
      if (next_msg)
        delete[] next_msg;
      next_msg_alloc_size = next_msg_size * 2;
      next_msg = new uint8_t[next_msg_alloc_size];
    }
    fread(next_msg, next_msg_size, 1, log);
    if (feof(log))
    {
      done = true;
      return false;
    }
    return true;
  }
  virtual const string __get_datatype() const { return string("*"); }
  virtual const string __get_md5sum()   const { return string("*"); }
  uint32_t serialization_length() { return next_msg_size; }
  virtual uint8_t *serialize(uint8_t *write_ptr)
  { 
    memcpy(write_ptr, next_msg, next_msg_size);
    return write_ptr + next_msg_size;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr) { assert(0); return NULL; }
};

class Playback : public node
{
public:
  Bag *bags;
  size_t num_bags;
  Playback(vector<string> bag_names) : node("playback"), num_bags(0)
  { 
    num_bags = bag_names.size();
    bags = new Bag[num_bags];
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
      int32_t min_usecs = 1234567890;
      for (size_t i = 0; i < num_bags; i++)
      {
        if (!bags[i].done)
        {
          keep_going = true;
          int32_t usecs = bags[i].usecs_to_next_msg();
          if (usecs < 5)
          {
            // send message
            publish(bags[i].topic_name, bags[i]);
            if (!bags[i].read_next_msg())
              bags[i].done = true;
            min_usecs = 0;
          }
          else if (usecs < min_usecs)
            min_usecs = usecs;
        }
      }
      if (!keep_going)
        break;
      if (min_usecs > 5)
        usleep(min_usecs - 5);
    }
  }
};

void sigint_handler(int sig)
{
  g_node->self_destruct();
}

int main(int argc, char **argv)
{
  signal(SIGINT, sigint_handler);
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
