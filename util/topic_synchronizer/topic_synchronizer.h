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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef TOPIC_SYNCHRONIZER_HH
#define TOPIC_SYNCHRONIZER_HH

#include "rosthread/mutex.h"
#include "rosthread/condition.h"
#include "ros/time.h"

  //! A templated class for synchronizing incoming topics
  /*! 
   * The Topic Synchronizer should be templated by your node, and is
   * passed a function pointer at construction to be called every time
   * all of your topics have arrived.
   */
template <class N>
class TopicSynchronizer
{

  class UnsubscribeList
  {
    std::list<std::string> list_;
    ros::thread::mutex list_mutex_;
    
  public:
    UnsubscribeList(std::list<std::string>& l) : list_(l) { }

    void doUnsubscribe(ros::node* n, std::string topic)
    {
      list_mutex_.lock();
      std::list<std::string>::iterator i = list_.begin();
      while (i != list_.end() && *i != topic)
        i++;
      
      if (i != list_.end())
      {
        i++;

        while (i != list_.end())
        {
          n->unsubscribe(*i);
          list_.erase(i++);
        }
      }
      list_mutex_.unlock();
    }
  };

  class UnsubscribeHelper
  {
    UnsubscribeList* ul_;
    std::string topic_;

  public:

    UnsubscribeHelper(UnsubscribeList* ul, std::string topic) : ul_(ul), topic_(topic) {} 

    void doUnsubscribe(ros::node* node)
    {
      ul_->doUnsubscribe(node, topic_);
    }
  };

  private:

  //! A pointer to your node for calling callback
  N* node_;

  //! The callback to be called if successful
  void (N::*callback_)(ros::Time);

  //! Timeout duration
  ros::Duration timeout_;

  //! The callback to be called if timed out
  void (N::*timeout_callback_)(ros::Time);
  
  //! The condition variable used for synchronization
  ros::thread::condition cond_all_;

  //! The number of expected incoming messages
  int expected_count_;

  //! The count of messages received so far
  int count_;

  //! Whether or not the given wait cycle has completed
  bool done_;

  //! Whether or not the node is ready to process data (all subscriptions are completed)
  bool ready_;

  //! The timestamp that is currently being waited for
  ros::Time waiting_time_;

  std::list< UnsubscribeList* > exclusion_lists_;
  std::list< UnsubscribeHelper** > helper_list_;

  //! The callback invoked by roscpp when any topic comes in
  /*!
   *  \param p  A void* which should point to the timestamp field of the incoming message
   */
  void msg_cb(void* p)
  {
    //Bail until we are actually ready
    if (!ready_)
      return;

    ros::Time* time = (ros::Time*)(p);

    cond_all_.lock();

    // If first to get message, wait for others
    if (count_ == 0)
    {
      wait_for_others(time);
      return;
    }

    // If behind, skip
    if (*time < waiting_time_)
    {
      cond_all_.unlock();
      return;
    }

    // If at time, increment count, possibly signal, and wait
    if (*time == waiting_time_)
    {
      count_++;
      if (count_ == expected_count_)
      {
        cond_all_.broadcast();
      }

      while (!done_ && *time == waiting_time_)
        cond_all_.wait();
      
      cond_all_.unlock();
      return;
    }

    // If ahead, wakeup others, and ten wait for others
    if (*time > waiting_time_)
    {
      cond_all_.broadcast();
      wait_for_others(time);
    }
  }

  void msg_unsubothers(void* p)
  {
    UnsubscribeHelper** uh = (UnsubscribeHelper**)(p);

    if (*uh != 0)
    {
      (*uh)->doUnsubscribe(node_);
      delete (*uh);
      (*uh) = 0;
    }
  }

  //! The function called in a message cb to wait for other messages
  /*!
   * \param time  The time that is being waited for
   */
  void wait_for_others(ros::Time* time)
  {
    count_ = 1;
    done_ = false;

    waiting_time_ = *time;
    bool timed_out = false;

    while (count_ < expected_count_ && *time == waiting_time_ && !timed_out)
      if (!cond_all_.timed_wait(timeout_))
      {
        timed_out = true;
        if (timeout_callback_)
          (*node_.*timeout_callback_)(*time);
      }

    if (*time == waiting_time_ && !timed_out)
      (*node_.*callback_)(*time);

    if (*time == waiting_time_)
    {
      done_ = true;
      count_ = 0;
      cond_all_.broadcast();
    }
    cond_all_.unlock();
  }

  public:

  //! Constructor
  /*! 
   * The constructor for the TopicSynchronizer
   *
   * \param node             A pointer to your node.
   * \param callback         A pointer to the callback to invoke when all messages have arrived
   * \param timeout          The duration 
   * \param timeout_callback A callback which is triggered when the timeout expires
   */
  TopicSynchronizer(N* node, void (N::*callback)(ros::Time), ros::Duration timeout = ros::Duration(1.0), void (N::*timeout_callback)(ros::Time) = NULL) : node_(node), callback_(callback), timeout_(timeout), timeout_callback_(timeout_callback), expected_count_(0), count_(0), done_(false)
  { }

  //! Destructor
  ~TopicSynchronizer()
  {
    for (typename std::list<UnsubscribeList*>::iterator i = exclusion_lists_.begin();
         i != exclusion_lists_.end();
         i++)
      delete (*i);

    for (typename std::list<UnsubscribeHelper**>::iterator i = helper_list_.begin();
         i != helper_list_.end();
         i++)
    {
      if (**i != 0)
        delete **i;
      delete (*i);
    }
  }

  //! Subscribe
  /*! 
   * The synchronized subscribe call.  Call this to subscribe for topics you want
   * to be synchronized.
   *
   * \param topic_name The name of the topic to subscribe to
   * \param msg        A reference to the message that will be populated on callbacks
   * \param queue_size The size of the incoming queue for the message
   */
  template <class M>
  void subscribe(std::string topic_name, M& msg, int queue_size)
  {
    node_->subscribe(topic_name, msg, &TopicSynchronizer<N>::msg_cb, this, &(msg.header.stamp), queue_size);
    expected_count_++;
  }

  template <class M>
  void subscribe(std::list<std::string> topic_names, M& msg, int queue_size)
  {
    UnsubscribeList* ul = new UnsubscribeList(topic_names);
    exclusion_lists_.push_back(ul); // so we can delete later

    for (std::list<std::string>::iterator tn_iter = topic_names.begin();
         tn_iter != topic_names.end();
         tn_iter++)
    {
      UnsubscribeHelper** uh = new UnsubscribeHelper*;
      *uh = new UnsubscribeHelper(ul, *tn_iter);
      helper_list_.push_back(uh);

      node_->subscribe(*tn_iter, msg, &TopicSynchronizer<N>::msg_unsubothers, this, uh, queue_size);
      node_->subscribe(*tn_iter, msg, &TopicSynchronizer<N>::msg_cb, this, &(msg.header.stamp), queue_size);
    }
    expected_count_++;
  }

  void ready()
  {
    ready_ = true;
  }
};

#endif
