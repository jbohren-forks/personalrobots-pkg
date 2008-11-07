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

#include "ros/node.h"
#include <string>
#include <map>

#include "logging/LogPlayer.h"
#include "logging/AnyMsg.h"

using namespace std;

struct LogContent
{
  string datatype;
  int count;
  LogContent(string d) : datatype(d), count(1) {}
};

map<string, LogContent> g_content;

void checkFile(string name, ros::msg* m, ros::Time t, void* n)
{
  map<string, LogContent>::iterator i = g_content.find(name);

  if (i == g_content.end())
  {
    g_content.insert(pair<string, LogContent>(name, LogContent(m->__get_datatype())));
  } else {
    i->second.count++;
  }
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("\nusage:\n");
    printf("checklog BAG\n");
    return 1;
  }

  LogPlayer player;

  if (player.open(string(argv[1]), ros::Time(0.0)))
  {
    player.addHandler<AnyMsg>(string("*"), &checkFile, NULL, false);
  }

  while(player.nextMsg())
  {
  }

  for (map<string, LogContent>::iterator i = g_content.begin();
       i != g_content.end();
       i++)
  {
    printf(" Name: %s  Datatype: %s  Count: %d\n", i->first.c_str(), i->second.datatype.c_str(), i->second.count);
  }

  return 0;
}
