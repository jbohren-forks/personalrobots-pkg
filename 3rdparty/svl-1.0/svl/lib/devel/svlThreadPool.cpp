/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2008, David Breeden
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlThreadPool.cpp
** AUTHOR(S):   David Breeden <breeden@cs.stanford.edu>
**
*****************************************************************************/

#include "svlThreadPool.h"

// Constructor
svlThreadPool::svlThreadPool(const unsigned size) {

#ifdef USE_PTHREADS
  _nThreads = size;
  _threads = new pthread_t[_nThreads];
  pthread_mutex_init(&_mutex, NULL);
  pthread_cond_init(&_cond, NULL);
  _workerArgs = new JobArgs;
#endif

}

// Destructor
svlThreadPool::~svlThreadPool() {

#ifdef USE_PTHREADS
  // tell threads to stop
  if (!_bQuit) {
    _bQuit = true;
    pthread_mutex_lock(&_mutex);
    pthread_cond_broadcast(&_cond);
    pthread_mutex_unlock(&_mutex);

    // wait for them to finish
    for (unsigned i = 0; i < _nThreads; i++) {
      pthread_join(_threads[i], NULL);
    }
  }

  pthread_mutex_destroy(&_mutex);
  pthread_cond_destroy(&_cond);
  delete[] _threads;
  delete _workerArgs;
#endif

}

// Spawn threads to take jobs
void svlThreadPool::start() {

#ifdef USE_PTHREADS
  _bQuit = false;

  _workerArgs->mutexPtr = &_mutex;
  _workerArgs->condPtr = &_cond;
  _workerArgs->jobQPtr = &_jobQ;
  _workerArgs->quitPtr = &_bQuit;

  for (unsigned i = 0; i < _nThreads; i++) {
    pthread_create(&_threads[i], NULL, runJobs, _workerArgs);
  }
#endif

}

// Add a new job to the job queue
void svlThreadPool::addJob(void *(*routine)(void *), void *arg) {

#ifdef USE_PTHREADS
  // Acquire lock on queue
  pthread_mutex_lock(&_mutex);

  // push job on queue
  _jobQ.push(make_pair(routine, arg));

  // Tell threads about it
  pthread_cond_broadcast(&_cond);

  // unlock queue
  pthread_mutex_unlock(&_mutex);
#endif

#ifdef USE_WINDOWSTHREADS
  // Just do it in the main thread.  TODO: actually implement
  routine(arg):
#endif
}

// Wait until jobs are finished, then return
void svlThreadPool::finish() {

#ifdef USE_PTHREADS
  while (true) {
    
    // Acquire lock on queue
    pthread_mutex_lock(&_mutex);

    // if it's empty, stop looping
    if (_jobQ.empty()) {
      break;
    }

    // if not, let the other threads get to it and try again
    pthread_mutex_unlock(&_mutex);
  }

  // tell the threads to quit
  _bQuit = true;
  pthread_cond_broadcast(&_cond);
  pthread_mutex_unlock(&_mutex);

  // Now wait for them to be done
  for (unsigned i = 0; i < _nThreads; i++) {
    pthread_join(_threads[i], NULL);
  }
#endif

}

// Thread main function
void *svlThreadPool::runJobs(void *argPtr) {

#ifdef USE_PTHREADS
  JobArgs *args = (JobArgs*) argPtr;

  // Keep asking for jobs until quit flag
  while (!*(args->quitPtr)) {

    // Wait for job
    pthread_mutex_lock(args->mutexPtr);
    while (args->jobQPtr->empty() && !*(args->quitPtr)) {
      pthread_cond_wait(args->condPtr, args->mutexPtr);
    }

    if (args->jobQPtr == NULL || args->jobQPtr->empty()) {
      pthread_mutex_unlock(args->mutexPtr);
      continue;
    }

    // Take job off queue
    pair<thread_routine_t, void *> job = args->jobQPtr->front();
    args->jobQPtr->pop();
    
    // unlock queue
    pthread_mutex_unlock(args->mutexPtr);
    
    // Do job
    job.first(job.second);
  }
#endif

  return NULL;
}
