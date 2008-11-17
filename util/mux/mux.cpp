///////////////////////////////////////////////////////////////////////////////
// The mux package provides a generic multiplexer
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
/////////////////////////////////////////////////////////////////////////////

#include "ros/node.h"
#include "std_msgs/String.h"
#include "std_srvs/StringString.h"
using namespace std;
using namespace ros;

// the type-safety of ROS is defeated by this message, since it uses
// the wildcard... TODO: figure out an elegant way to snarf the incoming 
// messages's md5sum so we can relay it to downstream connections.
// as on star trek, you've always got to be on the lookout for shape shifters

class ShapeShifter : public msg
{
public:
  uint8_t *msgBuf;
  uint32_t msgBufUsed, msgBufAlloc; 
  string topicName;
  ShapeShifter() : msg(), msgBuf(NULL), msgBufUsed(0), msgBufAlloc(0) { }
  virtual ~ShapeShifter() { if (msgBuf) delete[] msgBuf; 
                            msgBuf = NULL; msgBufAlloc = 0; }
  virtual const string __get_datatype() const { return string("*"); }
  virtual const string __get_md5sum()   const { return string("*"); }
  static const string __s_get_datatype() { return string("*"); }
  static const string __s_get_md5sum()   { return string("*"); }
  uint32_t serialization_length() { return msgBufUsed; }
  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t)
  {
    // yack up what we stored
    memcpy(writePtr, msgBuf, msgBufUsed);
    return writePtr + msgBufUsed;
  }
  virtual uint8_t *deserialize(uint8_t *readPtr)
  {
    // stash this message in our buffer
    if (__serialized_length > msgBufAlloc)
    {
      delete[] msgBuf;
      msgBuf = new uint8_t[__serialized_length];
      msgBufAlloc = __serialized_length;
    }
    msgBufUsed = __serialized_length;
    memcpy(msgBuf, readPtr, __serialized_length);
    return NULL;
  }
};

class Mux : public node
{
public:
  ShapeShifter *inMsgs;
  std_msgs::String topicSelMsg;
  ShapeShifter *selectedTopic;
  string outTopicName;
  uint32_t numInTopics;

  Mux(string outTopic, string selTopic, vector<string> inTopics) 
  : node("mux"), selectedTopic(NULL), outTopicName(outTopic)
  {
    subscribe(selTopic, topicSelMsg, &Mux::sel_cb);
    advertise<ShapeShifter>(outTopic);
    advertise_service(selTopic + string("Srv"), &Mux::selSrvCB);
    numInTopics = inTopics.size();
    inMsgs = new ShapeShifter[numInTopics];
    for (size_t i = 0; i < numInTopics; i++)
    {
      inMsgs[i].topicName = inTopics[i];
      subscribe(inTopics[i], inMsgs[i], &Mux::in_cb, &inMsgs[i]);
    }
    // by default, select the first topic
    if (numInTopics > 0)
      selectedTopic = &inMsgs[0];
  }
  virtual ~Mux()
  {
    delete[] inMsgs;
    inMsgs = NULL;
  }
  void in_cb(void *ssPtr)
  {
    if (ssPtr == selectedTopic)
      publish(outTopicName, *selectedTopic);
  }
  void sel_cb()
  {
    // spin through our vector of inputs and find this guy
    for (size_t i = 0; i < numInTopics; i++)
      if (inMsgs[i].topicName == topicSelMsg.data)
      {
        selectedTopic = &inMsgs[i];
        printf("mux selected input %d: [%s]\n", i, inMsgs[i].topicName.c_str());
        break;
      }
  }
  bool selSrvCB(std_srvs::StringString::request  &req,
                std_srvs::StringString::response &res)
  {
    if (selectedTopic)
      res.str = selectedTopic->topicName;
    else
      res.str = string("");
    // see if it's the magical '__none' topic, in which case we do an open circuit
    if (req.str == string("__none"))
    {
      printf("mux selected to no input.\n");
      selectedTopic = NULL;
      return true;
    }
    // spin through our vector of inputs and find this guy
    for (size_t i = 0; i < numInTopics; i++)
      if (inMsgs[i].topicName == req.str)
      {
        selectedTopic = &inMsgs[i];
        printf("mux selected input %d: [%s]\n", i, inMsgs[i].topicName.c_str());
        return true;
      }
    return false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 4)
  {
    printf("\nusage: mux OUT_TOPIC SEL_TOPIC IN_TOPIC1 [IN_TOPIC2 [...]]\n\n");
    return 1;
  }
  vector<string> topics;
  for (int i = 3; i < argc; i++)
    topics.push_back(argv[i]);
  Mux mux(argv[1], argv[2], topics);
  mux.spin();
  ros::fini();
  return 0;
}

