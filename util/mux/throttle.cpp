///////////////////////////////////////////////////////////////////////////////
// The mux package provides a generic multiplexer
//
// Copyright (C) 2008, Morgan Quigley, Brian Gerkey
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

/** @defgroup throttle throttle

throttle is a node that can subscribe to a topic and republish incoming
data to another topic, at a maximum rate.  It's mainly useful for
limiting bandwidth usage, e.g., over a wireless link.  It can work with
any message type.

<hr>

@section usage Usage
@verbatim
$ throttle <rate> <intopic> <outtopic> [standard ROS arguments] 
@endverbatim
Options:
- @b rate: Maximum republication rate, in Hz
- @b intopic: Incoming topic to subscribe to
- @b outtopic: Outgoing topic to publish on

Example, throttling bandwidth-hogging laser scans to 1Hz:
@verbatim
$ throttle 1.0 base_scan base_scan_throttled
@endverbatim
**/

#include <stdlib.h>
#include "ros/ros.h"
#include "ros/console.h"
using namespace std;

#define USAGE "USAGE: throttle <rate> <intopic> <outtopic>"

// the type-safety of ROS is defeated by this message, since it uses
// the wildcard... TODO: figure out an elegant way to snarf the incoming
// messages's md5sum so we can relay it to downstream connections.
// as on star trek, you've always got to be on the lookout for shape shifters

static bool g_initialized = false;
static string g_md5sum = "*", g_datatype = "*", g_message_definition = "*";
static uint8_t *msgBuf = NULL;
static uint32_t msgBufUsed=0, msgBufAlloc=0;

class ShapeShifter : public ros::Message
{
public:
  ShapeShifter() : Message() { }
  virtual ~ShapeShifter() { }
  virtual const string __getDataType() const { return g_datatype; }
  virtual const string __getMD5Sum()   const { return g_md5sum; }
  virtual const string __getMessageDefinition()   const { return g_message_definition; }
  static const string __s_getDataType() { return g_datatype; }
  static const string __s_getMD5Sum()   { return g_md5sum; }
  static const string __s_getMessageDefinition()   { return g_message_definition; }
  uint32_t serializationLength() const { return msgBufUsed; }
  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t) const
  {
    // yack up what we stored
    ROS_ASSERT(g_initialized);
    memcpy(writePtr, msgBuf, msgBufUsed);
    return writePtr + msgBufUsed;
  }
  virtual uint8_t *deserialize(uint8_t *readPtr)
  {
    // stash this message in our buffer
    if (__serialized_length > msgBufAlloc)
    {
      ROS_DEBUG("Allocating new buffer of size %u (old size: %u)", 
                __serialized_length, msgBufAlloc);
      delete[] msgBuf;
      msgBuf = new uint8_t[__serialized_length];
      msgBufAlloc = __serialized_length;
    }
    msgBufUsed = __serialized_length;
    memcpy(msgBuf, readPtr, __serialized_length);

    if(!g_initialized)
    {
      ROS_DEBUG("Storing message metadata");
      // Remember the md5 and type
      ros::M_string::iterator d = __connection_header->find(std::string("type"));
      if (d != __connection_header->end())
        g_datatype = d->second;
      else
        ROS_WARN("Connection header was not populated.\n");

      ros::M_string::iterator m = __connection_header->find(std::string("md5sum"));
      if (m != __connection_header->end())
        g_md5sum = m->second;
      else
        ROS_WARN("Connection header was not populated.\n");

      ros::M_string::iterator f = __connection_header->find(std::string("message_definition"));
      if (f != __connection_header->end())
        g_message_definition = f->second;
      else
        ROS_WARN("Could not find message_definition in connection header.\n");

      g_initialized = true;
    }

    return NULL;
  }
};

class Throttle
{
  public:
    Throttle(const ros::Duration& period,
             const string& intopic, 
             const string& outtopic) :
            period_(period),
            intopic_(intopic),
            outtopic_(outtopic)
    {
      sub = n.subscribe<ShapeShifter>(intopic_, 1, &Throttle::in_cb, this);
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    ros::Duration period_;
    string intopic_, outtopic_;
    ros::Time last_time_;

    void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
    {
      // Did we advertise yet?
      if(!pub)
        pub = n.advertise<ShapeShifter>(outtopic_, 1);

      ros::Time now = ros::Time::now();
      if((now - last_time_) > period_)
      {
        pub.publish(msg);
        last_time_ = now;
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "throttle", ros::init_options::AnonymousName);
  if (argc != 4)
  {
    puts(USAGE);
    return 1;
  }

  Throttle t(ros::Duration(1.0/atof(argv[1])), argv[2], argv[3]);

  ROS_INFO("Republishing %s to %s at a max rate of %.3f Hz",
           argv[2], argv[3], atof(argv[1]));

  ros::spin();

  return 0;
}

