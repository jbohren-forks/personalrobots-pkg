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
#include "logging/LogRecorder.h"
#include <string>

using namespace std;

class Vacuum : public ros::node
{
public:
  LogRecorder<> *bags;
  Vacuum(vector<string> vac_topics) : node("vacuum")
  { 
    bags = new LogRecorder<>[vac_topics.size()];
    time_t t = ::time(NULL);
    struct tm *tms = localtime(&t);
    char logdir[500];
    snprintf(logdir, sizeof(logdir), "%d-%02d-%02d-%02d-%02d-%02d",
             tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
             tms->tm_hour     , tms->tm_min  , tms->tm_sec);
    mkdir(logdir, 0755);
    ros::Time start = ros::Time::now();
    for (size_t i = 0; i < vac_topics.size(); i++)
    {
      string sanitized = vac_topics[i];
      printf("vacuum up [%s]\n", vac_topics[i].c_str());
      for (size_t j = 0; j < vac_topics[i].length(); j++)
      {
        char c = sanitized[j]; // sanitize it a bit
        if (c == '\\' || c == '/' || c == '#' || c == '&' || c == ';')
          sanitized[j] = '_';
      }
      if (!bags[i].open_log(std::string(logdir) + std::string("/") + sanitized + string(".bag"),
                            map_name(vac_topics[i]),
                            start))
        throw std::runtime_error("couldn't open log file\n");
      
      subscribe(vac_topics[i], bags[i], &Vacuum::dummy_cb);
    }
  }
  virtual ~Vacuum()
  {
    delete[] bags;
    bags = NULL;
  }
  void dummy_cb() { }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc <= 1)
  {
    printf("\nusage: vacuum TOPIC1 [TOPIC2] ...\n\n");
    return 1;
  }
  vector<std::string> topics;
  for (int i = 1; i < argc; i++)
    topics.push_back(argv[i]);
  Vacuum vac(topics);
  vac.spin();
  ros::fini();
  return 0;
}
