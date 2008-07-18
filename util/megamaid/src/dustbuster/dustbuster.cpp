///////////////////////////////////////////////////////////////////////////////
// The megamaid package provides logging and playback.
// Like vacuum, but can only hold one of each message.
//
// Copyright (C) 2008, Willow Garage Inc.
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


class dustbuster : public ros::node
{
public:
  dustbuster(vector<string> &vac_topics, string dirname, string filebase) : node("dustbuster")
  { 
    done = false;
    nTopics = vac_topics.size();
    bags = new LogRecorder<>[vac_topics.size()];
    //    time_t t = ::time(NULL);
    //    struct tm *tms = localtime(&t);
    mkdir(dirname.c_str(), 0755);

    ros::Time start = ros::Time::now();
    for (size_t i = 0; i < vac_topics.size(); i++)
    {

      //Find the next file name in dirname.
      int filenum=0;
      FILE *fp;
      string final;
      do {
	char prefix[200];
	sprintf(prefix, "%03d-%s-", filenum, filebase.c_str());
	string sanitized = std::string(prefix) + vac_topics[i];
	//Sanitize.
	for (size_t j = 0; j < vac_topics[i].length(); j++)
	  {
	    char c = sanitized[j]; // sanitize it a bit
	    if (c == '\\' || c == '/' || c == '#' || c == '&' || c == ';')
	      sanitized[j] = '_';
	  }
        final = dirname + std::string("/") + sanitized + std::string(".bag");
	filenum++;
      } while((fp = fopen(final.c_str(), "r")));
      //Now final contains the right filename.


      printf("dustbustering up [%s]\n", final.c_str());
      if (!bags[i].open_log(final,
                            map_name(vac_topics[i]),
                            start))
        throw std::runtime_error("couldn't open log file\n");

      subscribe(vac_topics[i], bags[i], &dustbuster::unsub_cb, &vac_topics[i]);
    }
  }

  virtual ~dustbuster()
  {
    delete[] bags;
    bags = NULL;
  }

  void unsub_cb(void *data) {
    static unsigned int nUnsubscribedFrom = 0;
    string *topic = static_cast<string*>(data);
    unsubscribe(*topic);
    cout << "Done with " << *topic << endl;
    nUnsubscribedFrom++;
    if(nUnsubscribedFrom == nTopics) {
      cout << "Collected one message of each.  Dustbuster is full." << endl;
      done = true;
    }
  }

  bool isDone() const {
    return done;
  }    

private:
  unsigned int nTopics;
  bool done;
  LogRecorder<> *bags;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 3)
  {
    printf("\nusage: dustbuster SAVEDIR FILENAME TOPIC [TOPIC] ... \n FILENAME will have a number and a .bag appended to it.\n\n");
    return 1;
  }

  string dirname = argv[1];
  string filebase = argv[2];
  cout << "Saving into " << dirname << "/ with base name " << filebase  << endl;

  vector<std::string> topics;
  for (int i = 3; i < argc; i++)
    topics.push_back(argv[i]);
  
  dustbuster *vac = new dustbuster(topics, dirname, filebase);
  while(!vac->isDone() && vac->ok()) {
    usleep(100);
  }

  ros::fini();
  delete vac;
  return 0;
}
