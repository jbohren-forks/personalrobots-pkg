/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <time.h>
#include <sys/stat.h>
#include "ros/node.h"
#include "logging/LogRecorder.h"
#include "logging/AnyMsg.h"
#include <string>
#include <rosthread/mutex.h>

using namespace std;

ros::thread::mutex unsub_mutex;
unsigned int nCallbacks;

void doUnsubscribe(string name, ros::msg* m, ros::Time t, void* n)
{
  unsub_mutex.lock();

  ((ros::node*)(n))->unsubscribe(name);

  cout << "Got a " << name << " message." << endl;
  nCallbacks++;

  if (((ros::node*)(n))->num_subscriptions() == nCallbacks) {
    cout << "Dustbuster is full." << endl;
    ((ros::node*)(n))->self_destruct();
  }
  unsub_mutex.unlock();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  string filename;
  vector<std::string> topics;

  // -- Vanilla usage case: just log a bunch of topics.
  if(argc > 2 && !strcmp(argv[1], "--vanilla")) {

    //Get the filename.
    time_t t = ::time(NULL);
    struct tm *tms = localtime(&t);
    char logdir[500];
    snprintf(logdir, sizeof(logdir), "%d-%02d-%02d-%02d-%02d-%02d",
	     tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
	     tms->tm_hour     , tms->tm_min  , tms->tm_sec);
    mkdir(logdir, 0755);
    filename = std::string(logdir) + std::string("/topics.bag");

    //Get the topics.
    for (int i = 2; i < argc; i++)
      topics.push_back(argv[i]);
  }


  // -- Dataset collection usage case.
  else if(argc > 4 && !strcmp(argv[1], "--seq")) {

    //Get the filename.
    mkdir(argv[2], 0755);
    string dirname = argv[2];
    string filebase = argv[3];

    int n=0;
    char num[3];
    FILE *fp;
    while(1) {
      sprintf(num, "%03d", n);
      filename = dirname + string("/") + num + string("-") + filebase + string(".bag");
      fp = fopen(filename.c_str(), "r");
      if(fp == NULL) {
	break;
      }
      fclose(fp);
      if(n>999) {
	cout << "too many logs" << endl;
	return 1;
      }
      n++;
    }

    //Get the topics.
    for (int i = 4; i < argc; i++)
      topics.push_back(argv[i]);
  }

  // -- Usage description case.
  else {
    printf("\nusage:\n\n"
	   "dustbuster --vanilla TOPIC1 [TOPIC2] ...\n"
	   "\t Saves to a timestamped directory.\n\n"
	   "dustbuster --seq SAVEDIR FILENAME TOPIC1 [TOPIC2] ... \n"
	   "\t For saving a sequence of logs.  FILENAME will have a number and a .bag appended to it.\n\n");
    return 1;
  }

  //Output confirmation of logging and filename.
  cout << "Logging topics ";
  for(unsigned int i=0; i<topics.size(); i++) {
    cout << "\"" << topics[i] << "\" ";
  }
  cout << " to file " << filename << endl;


  //Setup the logger.
  ros::node n("dustbuster");  // Ros peer style usage.
  LogRecorder l(&n);
  ros::Time start = ros::Time::now();

  if (l.open(filename, start))
  {
    for (vector<std::string>::iterator i = topics.begin(); i != topics.end(); i++)
    {
      printf("dustbustering up [%s]\n", i->c_str());
      l.addTopic<AnyMsg>(*i, 1, &doUnsubscribe, (void*)(&n), false);
    }

    l.start();
  } else {
    n.self_destruct();
  }

  //Wait for messages to log.
  cout << "Ready and waiting..." << endl;
  n.spin();
  ros::fini();
  return 0;
}
