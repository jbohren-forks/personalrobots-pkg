#ifndef _dustbuster_h_
#define _dustbuster_h_

#include <time.h>
#include <sys/stat.h>
#include "ros/node.h"
#include "logging/LogRecorder.h"
#include <string>


class dustbuster : public ros::node
{
 public:
  dustbuster(std::vector<std::string> &db_topics, std::string dirname, std::string filename);
  virtual ~dustbuster();
  bool isDone() const;

 private:
  void unsub_cb(void *data);
  unsigned int nTopics;
  bool done;
  LogRecorder<> *bags;
};


#endif
