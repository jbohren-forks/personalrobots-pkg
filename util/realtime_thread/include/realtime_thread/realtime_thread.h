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

#ifndef REALTIME_THREAD_H
#define REALTIME_THREAD_H

#include <native/mutex.h>
#include <native/cond.h>
#include <pthread.h>

typedef union {
  RT_MUTEX rt;
  pthread_mutex_t pt;
} RealtimeMutex;

typedef union {
  RT_COND rt;
  pthread_cond_t pt;
} RealtimeCond;

int realtime_mutex_create(RealtimeMutex *mutex);
int realtime_mutex_delete(RealtimeMutex *mutex);
int realtime_mutex_lock(RealtimeMutex *mutex);
int realtime_mutex_trylock(RealtimeMutex *mutex);
int realtime_mutex_unlock(RealtimeMutex *mutex);

int realtime_cond_create(RealtimeCond *cond);
int realtime_cond_delete(RealtimeCond *cond);
int realtime_cond_signal(RealtimeCond *cond);
int realtime_cond_wait(RealtimeCond *cond, RealtimeMutex *mutex);


#endif
