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
#include "std_msgs/String.h"

using namespace std;

void all_handler(string name, ros::msg* m, ros::Time t, void* n)
{
  int* counter = (int*)(n);

  cout << "all_handler saw a message named " << name << " with datatype: " << m->__get_datatype() << " Count at: " << (*counter)++ << endl;
}

void string_handler(string name, std_msgs::String* str, ros::Time t, void* n)
{
  cout << "string_handler saw a message named " << name << " with datatype: " << str->__get_datatype() << " and value: " << str->data << endl;
}

class LogHandler
{
public:
  int counter;

  LogHandler() : counter(0) {}

  void all_handler(string name, ros::msg* m, ros::Time t, void* n)
  {
    cout << "LogHandler::all_handler saw a message named " << name << " with datatype: " << m->__get_datatype() << " Count at: " << counter++ << endl;
  } 

  void string_handler(string name, std_msgs::String* str, ros::Time t, void* n)
  {
    cout << "LogHandler::string_handler saw a message named " << name << " with  datatype: " << str->__get_datatype() << " and value: " << str->data << endl;
  }
};

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    cout << endl << "usage:" << endl <<  " demo BAG" << endl;
    return 1;
  }

  LogPlayer player;
  LogHandler handler;

  int counter = 0;

  if (player.open(string(argv[1]), ros::Time(0.0)))
  {
    player.addHandler<AnyMsg>(string("*"), &all_handler, &counter, false);
    player.addHandler<std_msgs::String>(string("chatter"), &string_handler, NULL);
    player.addHandler<AnyMsg>(string("babble"), &LogHandler::all_handler, &handler, &counter, false);
    player.addHandler<std_msgs::String>(string("*"), &LogHandler::string_handler, &handler, NULL);
  }

  while(player.nextMsg())
  {
  }

  return 0;
}
