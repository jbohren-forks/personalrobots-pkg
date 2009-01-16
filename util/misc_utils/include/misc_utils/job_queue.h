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

/** @author Timothy Hunter <tjhunter@willowgarage.com>
*/

#ifndef REALTIME_JOB_QUEUE_H
#define REALTIME_JOB_QUEUE_H

#include <boost/thread/mutex.hpp>
#include <queue>
#include <cassert>

namespace misc_utils
{

class JobQueueItem
{
public:
  JobQueueItem(){}

  virtual void process() = 0;
};

/** @class JobQueue
  * @brief Implements a job that is protected against concurrent programming
  * This class is designed for use in a controller or any class looping in the real-time loop that receives input from the non real-time side of the system.
  * It should be included in a class  this way:
  */
class JobQueue
{
public:
  // The maximum size of the queue
  static const unsigned int MAX_QUEUE_SIZE = 100;

  JobQueue(){}

  //TODO: discuss policy when jobs are still in the queue
  ~JobQueue(){}

  /** @brief adds a job to the queue (blocking call)
    * @note the job will be managed (and deleted) by the queue once it is executed, hence one should not rely on this pointer once it is passed to the queue.
    */
  void push(JobQueueItem *job)
  {
    queue_lock_.lock();
    assert(queue_.size()<MAX_QUEUE_SIZE);
    queue_.push(job);
    queue_lock_.unlock();
  }


  /** @brief assigns the pointer job to the first element of the queue and removes it of the queue (blocking call)
    * @return a pointer to the first element, or NULL if the queue is empty
    */
  void get(JobQueueItem *job)
  {
    queue_lock_.lock();
    if(queue_.empty())
      job=NULL;
    else
    {
      job=queue_.front();
      queue_.pop();
    }
    queue_lock_.unlock();
  }

  /** @brief assigns the pointer job to the first element of the queue and removes it of the queue (non blocking call)
    * @return true if a job could be popped out of the queue and the passed pointer could be assigned, false in any other case.
    */
  bool tryGet(JobQueueItem *job)
  {
    if(queue_lock_.try_lock())
    {
      if(!queue_.empty())
      {
        job=queue_.front();
        queue_.pop();
        queue_lock_.unlock();
        return true;
      }
      queue_lock_.unlock();
    }
    return false;
  }

  /** @brief empties the queues by processing all the remaining jobs
    */
  void processAll()
  {
    queue_lock_.lock();
    while(!queue_.empty())
    {
      JobQueueItem *job=queue_.front();
      queue_.pop();
      job->process();
      delete job;
    }
    queue_lock_.unlock();
  }

  /** @brief empties the queues without processing all the remaining jobs
    */
  void clear()
  {
    queue_lock_.lock();
    while(!queue_.empty())
    {
      JobQueueItem *job=queue_.front();
      queue_.pop();
      delete job;
    }
    queue_lock_.unlock();
  }

private:
  std::queue<JobQueueItem *> queue_;
  boost::mutex queue_lock_;  // Protects queue_

};
/*
    pthread_cond_init(&updated_cond_, NULL);
    if (0 != pthread_mutex_init(&msg_lock_, NULL))
    {
      perror("pthread_mutex_init");
      abort();
    }
    keep_running_ = true;
    thread_ = ros::thread::member_thread::startMemberFunctionThread<RealtimePublisher<Msg> >
      (this, &RealtimePublisher::publishingLoop);
  }

  ~RealtimePublisher()
  {
    node_->unadvertise(topic_);
  }

  void stop()
  {
    keep_running_ = false;
  }

  Msg msg_;

  int lock()
  {
    return pthread_mutex_lock(&msg_lock_);
  }

  bool trylock()
  {
    if (0 == pthread_mutex_trylock(&msg_lock_))
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        pthread_mutex_unlock(&msg_lock_);
        return false;
      }
    }
    else
      return false;
  }

  void unlock()
  {
    pthread_mutex_unlock(&msg_lock_);
  }

  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    pthread_mutex_unlock(&msg_lock_);
    pthread_cond_signal(&updated_cond_);
  }

  bool is_running() const { return is_running_; }

  void publishingLoop()
  {
    is_running_ = true;
    turn_ = REALTIME;
    while (keep_running_)
    {
      // Locks msg_ and copies it
      pthread_mutex_lock(&msg_lock_);
      while (turn_ != NON_REALTIME)
        pthread_cond_wait(&updated_cond_, &msg_lock_);
      Msg outgoing(msg_);
      turn_ = REALTIME;
      pthread_mutex_unlock(&msg_lock_);

      // Sends the outgoing message
      node_->publish(topic_, outgoing);
    }
    is_running_ = false;
  }

private:

  std::string topic_;
  ros::Node *node_;
  bool is_running_;
  bool keep_running_;

  pthread_t *thread_;

  pthread_mutex_t msg_lock_;  // Protects msg_
  pthread_cond_t updated_cond_;

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use msg_?
};*/

}

#endif
