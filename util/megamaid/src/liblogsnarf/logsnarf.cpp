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

LogSnarfer::LogSnarfer()
{}

LogSnarfer::~LogSnarfer()
{ }

void LogSnarfer::addLog(LogPlayerBase &l, string file_name, void (*fp)(ros::Time))
{
  l.open_log(file_name, ros::Time(0,0), true);
  LogAndCallback lcb;
  lcb.log = &l;
  lcb.fp  = fp;
  logs.push_back(lcb);
}

void LogSnarfer::snarf()
{
  bool keep_going = true;
  while (keep_going)
  {
    bool keep_going = false;
    ros::Time min_t(-1,0);
    vector<LogAndCallback>::iterator l;
    for (vector<LogAndCallback>::iterator i = logs.begin(); i != logs.end(); i++)
    {
      if (!i->log->done)
      {
        keep_going = true;
        if (i->log->time_of_msg() < min_t)
        {
          l = i;
          min_t = i->log->time_of_msg();
        }
      }
    }

    if (!keep_going)
      return;

    (*(l->fp))(min_t);

    l->log->read_next_msg(true);
  }
};

