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

#if defined(__APPLE__)
#define HAS_XENOMAI 0
#else
#define HAS_XENOMAI 1
#endif

#include "realtime_tools/realtime_tools.h"
#if HAS_XENOMAI
#include "realtime_tools/xenomai_prototypes.h"
#endif

int realtime_mutex_create(RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_mutex_create)
    err = rt_mutex_create(&mutex->rt, NULL);
  else
#endif
    err = pthread_mutex_init(&mutex->pt, NULL);

  return err;
}

int realtime_mutex_delete(RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_mutex_delete)
    err = rt_mutex_delete(&mutex->rt);
  else
#endif
    err = pthread_mutex_destroy(&mutex->pt);

  return err;
}

int realtime_mutex_lock(RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_mutex_acquire)
    err = rt_mutex_acquire(&mutex->rt, TM_INFINITE);
  else
#endif
    err = pthread_mutex_lock(&mutex->pt);

  return err;
}

int realtime_mutex_trylock(RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_mutex_acquire)
    err = rt_mutex_acquire(&mutex->rt, TM_NONBLOCK);
  else
#endif
    err = pthread_mutex_trylock(&mutex->pt);

  return err;
}

int realtime_mutex_unlock(RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_mutex_release)
    err = rt_mutex_release(&mutex->rt);
  else
#endif
    err = pthread_mutex_unlock(&mutex->pt);

  return err;
}


int realtime_cond_create(RealtimeCond *cond)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_cond_create)
    err = rt_cond_create(&cond->rt, NULL);
  else
#endif
    err = pthread_cond_init(&cond->pt, NULL);

  return err;
}

int realtime_cond_delete(RealtimeCond *cond)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_cond_delete)
    err = rt_cond_delete(&cond->rt);
  else
#endif
    err = pthread_cond_destroy(&cond->pt);

  return err;
}

int realtime_cond_signal(RealtimeCond *cond)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_cond_signal)
    err = rt_cond_signal(&cond->rt);
  else
#endif
    err = pthread_cond_signal(&cond->pt);

  return err;
}

int realtime_cond_wait(RealtimeCond *cond, RealtimeMutex *mutex)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_cond_wait)
    err = rt_cond_wait(&cond->rt, &mutex->rt, TM_INFINITE);
  else
#endif
    err = pthread_cond_wait(&cond->pt, &mutex->pt);

  return err;
}

int realtime_shadow_task(RealtimeTask *task)
{
  int err = 0;

#if HAS_XENOMAI
  if (rt_task_shadow)
    err = rt_task_shadow(&task->rt, NULL, 0, 0);
#endif

  // No pthread equivalent

  return err;
}

double realtime_gettime(void)
{
  double now = 0;

#if HAS_XENOMAI
  if (rt_timer_read)
  {
    RTIME n = rt_timer_read();
    now = double(n) / 1e9;
  }
  else
#endif
  {
    struct timespec n;
    clock_gettime(CLOCK_REALTIME, &n);
    now = double(n.tv_nsec) / 1e9 + n.tv_sec;
  }

  return now;
}
