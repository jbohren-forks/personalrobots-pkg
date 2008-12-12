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

/** \author Josh Faust */

#include <ros/node.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <string>
#include <list>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

/// \todo remove backward compatability
#include "tf/TransformArray.h"

namespace tf
{

/**
 * \brief For internal use only.
 *
 * In general, allocating memory inside a library's header using new/delete is a good way to cause difficult to find bugs.
 * This provides a way for MessageNotifier to allocate from within the tf library, rather than within the calling library.
 */
void* notifierAllocate(uint32_t size);
/**
 * \brief For internal use only.
 *
 * In general, allocating memory inside a library's header using new/delete is a good way to cause difficult to find bugs.
 * This provides a way for MessageNotifier to allocate from within the tf library, rather than within the calling library.
 */
void notifierDeallocate(void* p);

class Transformer;

/**
 * \class MessageNotifier
 * \brief Queues messages that include a Header until there is transform data available for the time the timestamp on the message.
 *
 * For use instead of extrapolation, MessageNotifier provides a way of waiting until it is possible to transform a message
 * before getting a callback notifying that the message is available.
 *
 * \section callback THE CALLBACK
 * The callback takes one argument, which is a boost shared_ptr to a message.  MessageNotifier provides a MessagePtr typedef,
 * so the signature of the callback is:
 \verbatim
 void funcName(const MessageNotifier<Message>::MessagePtr& message);
 \endverbatim
 *
 * A bare function can be passed directly as the callback.  Methods must be passed using boost::bind.  For example
 \verbatim
 boost::bind(&MyObject::funcName, this, _1);
 \endverbatim
 *
 * The message is \b not locked when your callback is invoked.
 *
 * \section threading THREADING
 * MessageNotifier spins up a single thread to call your callback from, so that it's possible to do a lot of work in your callback
 * without blocking the rest of the application.
 *
 \endverbatim
 */
template<class Message>
class MessageNotifier
{
public:
  typedef boost::shared_ptr<Message> MessagePtr;
  typedef boost::function<void(const MessagePtr&)> Callback;

  /**
   * \brief Constructor
   * \param tf The Transformer to use for checking if the transform data is available
   * \param node The ros::node to subscribe on
   * \param callback The function to call when a message is ready to be transformed
   * \param topic The topic to listen on
   * \param target_frame The frame we need to be able to transform to before a message is ready
   * \param queue_size The number of messages to keep around waiting for transform data.  This is passed directly to ros::node::subscribe.
   * \note A queue size of 0 means infinite, which is dangerous
   */
  MessageNotifier(Transformer* tf, ros::node* node, Callback callback,
      const std::string& topic, const std::string& target_frame,
      uint32_t queue_size)
  : tf_(tf)
  , node_(node)
  , callback_(callback)
  , target_frame_(target_frame)
  , queue_size_(queue_size)
  , message_count_(0)
  , destructing_(false)
  , new_messages_(false)
  , new_transforms_(false)
  , successful_transform_count_(0)
  , failed_transform_count_(0)
  , transform_message_count_(0)
  , incoming_message_count_(0)
  {
    setTopic(topic);

    node_->subscribe("/tf_message", transforms_message_,
        &MessageNotifier::incomingTFMessage, this, 1);
    node_->subscribe("/TransformArray", old_transforms_message_,
        &MessageNotifier::incomingOldTFMessage, this, 1);

    thread_handle_ = new boost::thread(boost::bind(
        &MessageNotifier::workerThread, this));
  }

  /**
   * \brief Destructor
   */
  ~MessageNotifier()
  {
    unsubscribeFromMessage();

    node_->unsubscribe("/tf_message", &MessageNotifier::incomingTFMessage, this);
    node_->unsubscribe("/TransformArray",
        &MessageNotifier::incomingOldTFMessage, this);

    // Tell the worker thread that we're destructing
    destructing_ = true;
    new_data_.notify_all();

    // Wait for the worker thread to exit
    thread_handle_->join();

    delete thread_handle_;

    clear();

    //printf("MessageNotifier: Successful Transforms: %d, Failed Transforms: %d, Transform messages received: %d, Messages received: %d\n",
    //       successful_transform_count_, failed_transform_count_, transform_message_count_, incoming_message_count_);
  }

  /**
   * \brief Set the frame you need to be able to transform to before getting a message callback
   */
  void setTargetFrame(const std::string& target_frame)
  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    target_frame_ = target_frame;
    new_data_.notify_all();
  }

  /**
   * \brief Set the topic to listen on
   */
  void setTopic(const std::string& topic)
  {
    unsubscribeFromMessage();

    topic_ = topic;

    subscribeToMessage();
  }

  /**
   * \brief Clear any messages currently in the queue
   */
  void clear()
  {
    boost::mutex::scoped_lock list_lock(list_mutex_);
    boost::mutex::scoped_lock queue_lock(queue_mutex_);

    messages_.clear();
    new_message_queue_.clear();
    message_count_ = 0;
  }

private:

  typedef std::vector<MessagePtr> V_Message;
  typedef std::list<MessagePtr> L_Message;

  /**
   * \brief Gather any messages ready to be transformed
   * \param to_notify Filled with the messages ready to be transformed, in the order they were received
   * \note Assumes the message list is already locked
   */
  void gatherReadyMessages(V_Message& to_notify)
  {
    to_notify.reserve(message_count_);

    int i = 0;

    typename L_Message::iterator it = messages_.begin();
    typename L_Message::iterator end = messages_.end();
    for (; it != end; ++i)
    {
      MessagePtr& message = *it;

      if (tf_->canTransform(target_frame_, message->header.frame_id, message->header.stamp))
      {
        // If we get here the transform succeeded, so push the message onto the notify list, and erase it from or message list
        to_notify.push_back(message);

        it = messages_.erase(it);
        --message_count_;

        //printf("Message %d ready, count now %d\n", i, message_count_);

        ++successful_transform_count_;
      }
      else
      {
        ++it;
        ++failed_transform_count_;
      }
    }
  }

  /**
   * \brief Calls the notification callback on each message in the passed vector
   * \param to_notify The list of messages to call the callback on
   */
  void notify(V_Message& to_notify)
  {
    typename V_Message::iterator it = to_notify.begin();
    typename V_Message::iterator end = to_notify.end();
    for (; it != end; ++it)
    {
      callback_(*it);
    }
  }

  /**
   * \brief Adds messages into the message list, removing old messages if necessary
   */
  void processNewMessages(V_Message& messages)
  {
    typename V_Message::iterator it = messages.begin();
    typename V_Message::iterator end = messages.end();
    for (; it != end; ++it)
    {
      MessagePtr& message = *it;

      // If this message is about to push us past our queue size, erase the oldest message
      if (message_count_ + 1 > queue_size_)
      {
        messages_.pop_front();
        --message_count_;

        //printf("Removed old message, count now %d\n", message_count_);
      }

      // Add the message to our list
      messages_.push_back(message);
      ++message_count_;

      //printf("Added message, count now %d\n", message_count_);
    }
  }

  /**
   * \brief Entry point into the worker thread that does all our actual work, including calling the notification callback
   */
  void workerThread()
  {
    V_Message to_notify;
    while (!destructing_)
    {
      V_Message local_queue;

      {
        boost::mutex::scoped_lock lock(queue_mutex_);

        // Wait for new data to be available
        while (!destructing_ && ((message_count_ == 0 && new_message_queue_.size() == 0) || (!new_transforms_ && !new_messages_)))
        {
          new_data_.wait(lock);
        }

        //printf("workerThread: woken, message count: %d\n", message_count_);

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
        boost::mutex::scoped_lock lock(list_mutex_);
        processNewMessages(local_queue);

        local_queue.clear();

        gatherReadyMessages(to_notify);

        new_transforms_ = false;

        notify(to_notify);
        to_notify.clear();
      }
    }
  }

  /**
   * \brief Subscribe to the message topic
   */
  void subscribeToMessage()
  {
    if (!topic_.empty())
    {
      node_->subscribe(topic_, message_, &MessageNotifier::incomingMessage,
          this, queue_size_);
    }
  }

  /**
   * \brief Unsubscribe from the message topic
   */
  void unsubscribeFromMessage()
  {
    if (!topic_.empty())
    {
      node_->unsubscribe(topic_, &MessageNotifier::incomingMessage, this);
    }
  }

  /**
   * \class MessageDeleter
   * \brief Since we allocate with our own special functions, we need to also delete using them.  This provides a deletion interface for the boost::shared_ptr
   */
  class MessageDeleter
  {
  public:
    void operator()(Message* m)
    {
      m->~Message();
      notifierDeallocate(m);
    }
  };

  /**
   * \brief Callback that happens when we receive a message on the message topic
   */
  void incomingMessage()
  {
    //printf("incoming message\n");
    // Allocate our new message and placement-new it
    Message* mem = (Message*) notifierAllocate(sizeof(Message));
    new (mem) Message();

    // Create a boost::shared_ptr from the message, with our custom deleter
    MessagePtr message(mem, MessageDeleter());
    // Copy the message
    *message = message_;

    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      new_message_queue_.push_back(message);

      //printf("Added message to message queue, count now %d\n", new_message_queue_.size());

      new_messages_ = true;

      // Notify the worker thread that there is new data available
      new_data_.notify_all();
    }

    ++incoming_message_count_;
  }

  /**
   * \brief Callback that happens when we receive a message on the TF message topic
   */
  void incomingTFMessage()
  {
    ////printf("incoming transforms\n");
    // Notify the worker thread that there is new data available
    new_data_.notify_all();
    new_transforms_ = true;
    ++transform_message_count_;
  }

  /**
   * \brief Callback that happens when we receive a message on the old TF TransformArray message topic
   */
  void incomingOldTFMessage()
  {
    ////printf("incoming transforms\n");
    // Notify the worker thread that there is new data available
    new_data_.notify_all();
    new_transforms_ = true;
    ++transform_message_count_;
  }

  Transformer* tf_; ///< The Transformer used to determine if transformation data is available
  ros::node* node_; ///< The node used to subscribe to the topic
  Callback callback_; ///< The callback to call when a message is ready
  std::string target_frame_; ///< The frame we need to be able to transform to before a message is ready
  std::string topic_; ///< The topic to listen on
  uint32_t queue_size_; ///< The maximum number of messages we queue up

  L_Message messages_; ///< The message list
  uint32_t message_count_; ///< The number of messages in the list.  Used because messages_.size() has linear cost
  boost::mutex list_mutex_; ///< The mutex used for locking message list operations

  Message message_; ///< The incoming message

  tfMessage transforms_message_; ///< The incoming TF transforms message
  tf::TransformArray old_transforms_message_; ///< The incoming old TF (rosTF) TransformArray message

  bool destructing_; ///< Used to notify the worker thread that it needs to shutdown
  boost::thread* thread_handle_; ///< Thread handle for the worker thread
  boost::condition_variable new_data_; ///< Condition variable used for waking the worker thread
  bool new_messages_; ///< Used to skip waiting on new_data_ if new messages have come in while calling back
  volatile bool new_transforms_; ///< Used to skip waiting on new_data_ if new transforms have come in while calling back or transforming data
  V_Message new_message_queue_; ///< Queues messages to later be processed by the worker thread
  boost::mutex queue_mutex_; ///< The mutex used for locking message queue operations

  int successful_transform_count_;
  int failed_transform_count_;
  int transform_message_count_;
  int incoming_message_count_;
};

} // namespace tf
