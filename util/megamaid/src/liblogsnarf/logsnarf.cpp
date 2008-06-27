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
#include "logsnarf/logsnarf.h"
using namespace ros;
using namespace std;

ros::Time start;

class Bag 
{
public:
  FILE *log;
  ros::Duration next_msg_dur;
  string topic_name;
  uint8_t *next_msg;
  uint32_t next_msg_size, next_msg_alloc_size;
  bool done;
  Bag() : log(NULL), next_msg(NULL), next_msg_alloc_size(0), 
          done(false) { }
  virtual ~Bag() { fclose(log); if (next_msg) delete[] next_msg; }
  bool open_log(const string &bag_name)
  {
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
};

LogSnarfer::LogSnarfer(const vector<string> &bag_names) : num_bags(0)
{ 
  num_bags = bag_names.size();
  bags = new Bag[num_bags];
  for (size_t i = 0; i < num_bags; i++)
  {
    printf("%s\n", bag_names[i].c_str());
    if (!bags[i].open_log(bag_names[i]))
      throw std::runtime_error("couldn't open bag file\n");
  }
}

LogSnarfer::~LogSnarfer()
{
  delete[] bags;
  bags = NULL;
}

bool LogSnarfer::snarf_one_message(std::string *topic, double *rel_time,
                                   uint8_t **data, uint32_t *data_len)
{
  double min_time = 1e100;
  int min_idx = -1;
  bool keep_going = false;
  for (size_t i = 0; i < num_bags; i++)
  {
    if (!bags[i].done)
    {
      keep_going = true;
      if (bags[i].next_msg_dur.to_double() < min_time)
      {
        min_idx = i;
        min_time = bags[i].next_msg_dur.to_double();
      }
    }
  }
  if (!keep_going)
    return false;
  assert(min_idx >= 0);
  Bag *b = &bags[min_idx];
  *topic = b->topic_name;
  *rel_time = min_time;
  *data = b->next_msg;
  *data_len = b->next_msg_size;
  if (!b->read_next_msg())
    b->done = true;
  return true;
};

