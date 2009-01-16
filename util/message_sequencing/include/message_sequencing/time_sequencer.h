/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jeremy Leibs and Josh Faust*/

#ifndef TIME_SEQUENCER_HH
#define TIME_SEQUENCER_HH

#include "message_sequencing.h"

#include <ros/node.h>

#include <sys/types.h>
#include <stdint.h>

#include <string>
#include <list>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace message_sequencing
{

/**
 * \class TimeSequencer
 *
 * \brief Sequences messages based on the timestamp of their header.
 *
 * \section behavior BEHAVIOR

 * At construction, the TimeSequencer takes a ros::Duration
 * "delay_time" which specifies how long to queue up messages to
 * provide a time sequencing over them.  As messages arrive they are
 * sorted according to their timestamps.  A callback for a message is
 * never invoked until the messages time_stamp is out of date by at
 * least delay_time.  However, for all messages which are out of date
 * by at least delay_time, their callback are invoked and guaranteed
 * to be in temporal order.  If a message arrives from a time \b prior
 * to a message which has already had its callback invoked, it is
 * instead passed to an optional separate callback for out of date
 * messages.
 *
 * \section callbacks THE CALLBACKS
 * The callbacks takes one argument, which is a boost shared_ptr to the message.
 *  \verbatim 
 *  void funcName(const boost:shared_ptr<MessageType>& message);
 *  \endverbatim
 *
 * A bare function can be passed directly as the callback.
 * Alternatively, class methods should be passed using boost::bind.  For
 * example:
 *  \verbatim
 *  boost::bind(&MyObject::funcName, this, _1);
 *  \endverbatim
 *
 * The message is \b not locked when your callback is invoked.
 *
 * \section threading THREADING
 * TimeSequencer spins up a single thread to call your callback from,
 * so that it's possible to do a lot of work in your callback without
 * blocking the rest of the application.
 *
 */
template<class Message>
class TimeSequencer : public ros::msg
{
public:
  typedef boost::shared_ptr<Message> MessagePtr;
  typedef boost::function<void(const MessagePtr&)> Callback;

  /**
   * \brief Since we allocate with our own special functions, we need
   * to also delete using them.  This provides a deletion interface
   * for the boost::shared_ptr
   */
  class MessageDeleter
  {
  public:
    void operator()(Message* m)
    {
      m->~Message();
      memDeallocate(m);
    }
  };

  /**
   * \brief Constructor
   * \param node The ros::Node to subscribe on
   * \param topic The topic to listen on
   * \param callback The function to call when a message is available
   * \param callback The function to call when a message is out of date
   * \param delay The maximum amount of time to wait 
   * \param buffer_queue_size The maximum number of messages to keep
   *   around while waiting for delay to pass.  When this number is
   *   exceeded, older messages are thrown out.
   * \param ros_queue_size The size of the incoming ros buffer.  This
   *   passes through to subscribe.
   */
  TimeSequencer(ros::Node* node, const std::string& topic, 
                Callback callback, Callback old_callback,
                ros::Duration delay,
                uint32_t buffer_queue_size,
                uint32_t ros_queue_size)
  : node_(node)
  , topic_(topic)
  , callback_(callback)
  , old_callback_(old_callback)
  , neg_delay_(ros::Duration().fromNSec(-delay.toNSec())) // Convert delay to be negative so we can add it easily
  , buffer_queue_size_(buffer_queue_size)
  , ros_queue_size_(ros_queue_size)
  , destructing_(false)
  , new_messages_(false)
  , last_time_(ros::Time().fromSec(0.0))
  {
    node_->subscribe(topic_, *this, &TimeSequencer::dummyCb, this, ros_queue_size_);

    thread_handle_ = new boost::thread(boost::bind(&TimeSequencer::workerThread, this));
  }

  /**
   * \brief Destructor
   */
  ~TimeSequencer()
  {
    node_->unsubscribe(topic_, &TimeSequencer::dummyCb, this);

    // Tell the worker thread that we're destructing
    destructing_ = true;
    new_data_.notify_all();

    // Wait for the worker thread to exit
    thread_handle_->join();

    delete thread_handle_;

    clear();
  }

  /**
   * \brief This callback is used by ROS but we don't need it since
   * we've hijacked the deserialized call.
   */
  void dummyCb()
  {
    
  }

  /**
   * \brief Clear any messages currently in the queue
   */
  void clear()
  {
    boost::mutex::scoped_lock list_lock(set_mutex_);
    boost::mutex::scoped_lock queue_lock(queue_mutex_);

    messages_.clear();
    new_message_queue_.clear();
  }

  // Functions to allow the message_buffer to pose as another message
  // This is what allows us to bypass an unnecessary copy when being
  // deserialized from the wire.

  inline static std::string __s_get_datatype() { return Message::__s_get_datatype(); }
  inline static std::string __s_get_md5sum() { return Message::__s_get_md5sum(); }

  virtual const std::string __get_datatype() const { return Message::__s_get_datatype(); }
  virtual const std::string __get_md5sum()   const { return Message::__s_get_md5sum(); }

  // Topic buffer is for subscribing, not publishing
  virtual uint32_t serialization_length() { return 0; }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t) { assert(0); return NULL; }

  /**
   * \brief Deserialize is called by ros on incoming messages.
   */
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    // Performance could likely be improved using a managed pool of messages
    // instead of constantly reallocating
    Message* mem = (Message*) memAllocate(sizeof(Message));
    new (mem) Message();

    // Create a boost::shared_ptr from the message, with our custom deleter
    MessagePtr message(mem, MessageDeleter());

    // Deserialize into our newly created message:
    message->__serialized_length = __serialized_length;
    uint8_t* ret = message->deserialize(read_ptr);
    
    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      new_message_queue_.push_back(message);

      new_messages_ = true;

      new_data_.notify_all();
    }

    return ret;
  }

private:

  /**
   * \brief The sorting operator for our message set.
   */
  class MessageSort
  {
  public:
    bool operator()(const MessagePtr& a, const MessagePtr& b) const
    {
      return a->header.stamp < b->header.stamp;
    }
  };

  typedef std::vector<MessagePtr> V_Message;
  typedef std::multiset<MessagePtr, MessageSort> S_Message;


  /**
   * \brief Dispatch the next message to the callback, including
   */
  ros::Time dispatchNextMessage()
  {


    // Purge all out of date messages:
    typename S_Message::iterator it = messages_.begin();
    typename S_Message::iterator end = messages_.end();
    while (it != end && (*it)->header.stamp < last_time_)
    {
      if (old_callback_)
        old_callback_(*it);
      ++it;
    }
    messages_.erase(messages_.begin(), it);


    // If there is a message left at the beginning execute it if it's time
    it = messages_.begin();

    if (it != end && (*it)->header.stamp < ros::Time::now() + neg_delay_)
    {
      last_time_ = (*it)->header.stamp;

      if (callback_)
        callback_(*it);

      messages_.erase(it);
    }

    it = messages_.begin();
    if (it != end)
    {
      // Return timestamp of next message if available
      return (*it)->header.stamp;
    } else {
      // Otherwise return maximum time in future  (This code will break in the year 2554)
      return ros::Time().fromNSec(UINT64_MAX);
    }
  }

  /**
   * \brief Adds messages into the message set
   */
  void processNewMessages(V_Message& new_messages)
  {
    typename V_Message::iterator it = new_messages.begin();
    typename V_Message::iterator end = new_messages.end();
    for (; it != end; ++it)
    {
      MessagePtr& message = *it;

      // If there is still room in our set, add it
      if (messages_.size() + 1 < buffer_queue_size_)
      {
        messages_.insert(message);
      } else {

        // As long as the message is more recent than the oldest message
        
        typename S_Message::iterator oldest = messages_.begin();
        if (oldest != messages_.end() && message->header.stamp > (*oldest)->header.stamp)
        {
          messages_.erase(oldest);
          messages_.insert(message);
        }
      }
    }
  }

  /**
   * \brief Entry point into the worker thread that does all our actual work, including calling the notification callback
   */
  void workerThread()
  {

    V_Message local_queue;

    ros::Time next_time = ros::Time().fromNSec(UINT64_MAX);

    while (!destructing_)
    {
      {
        boost::mutex::scoped_lock lock(queue_mutex_);

        ros::Duration time_to_go = next_time - (ros::Time::now() + neg_delay_);

        // Wait for new data to be available or time of next message to be reached
        while (!destructing_                                // While we are not destructing
               && !new_messages_                            // And there are no new messages
               && ros::Duration().fromNSec(0) < time_to_go) // And time_to_go is positive
        {
          boost::posix_time::time_duration timeout;
          // Never wait for more than 1/10th of a second
          if (time_to_go < ros::Duration().fromSec(0.1))
            timeout = boost::posix_time::nanosec(time_to_go.toNSec());
          else
            timeout = boost::posix_time::milliseconds(100);

          new_data_.timed_wait(lock, timeout);
          
          time_to_go = next_time - (ros::Time::now() + neg_delay_);
        }

        // If we're destructing, break out of the loop
        if (destructing_)
        {
          break;
        }

        local_queue.swap(new_message_queue_);
        new_messages_ = false;
      }

      {
        // Outside the queue lock, gather and notify that the messages are ready
        // Need to lock the list mutex because clear() can modify the message list
        boost::mutex::scoped_lock lock(set_mutex_);
        processNewMessages(local_queue);

        local_queue.clear();

        // We only dispatch one message at a time so we can swap in new messages that arrived
        // during the processing time.
        next_time = dispatchNextMessage();
      }
    }
  }

  ros::Node* node_; ///< The node used to subscribe to the topic
  std::string topic_; ///< The topic to listen on
  Callback callback_; ///< The callback to call when a message is ready
  Callback old_callback_; ///< The callback to call when a message is out of date
  ros::Duration neg_delay_;
  uint32_t buffer_queue_size_; ///< The maximum number of messages we queue up
  uint32_t ros_queue_size_; ///< The maximum number of messages ROS will queue up

  S_Message messages_; ///< The message set
  boost::mutex set_mutex_; ///< The mutex used for locking message list operations

  bool destructing_; ///< Used to notify the worker thread that it needs to shutdown
  boost::thread* thread_handle_; ///< Thread handle for the worker thread

  boost::condition_variable new_data_; ///< Condition variable used for waking the worker thread
  bool new_messages_; ///< Used to skip waiting on new_data_ if new messages have come in while calling back

  V_Message new_message_queue_; ///< Queues messages to later be processed by the worker thread
  boost::mutex queue_mutex_; ///< The mutex used for locking message queue operations


  ros::Time last_time_; ///< The last time at which we processed a message
};

} // Namespace: message_sequencing

#endif
