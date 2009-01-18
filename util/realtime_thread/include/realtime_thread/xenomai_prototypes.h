/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef XENOMAI_PROTOTYPES_H
#define XENOMAI_PROTOTYPES_H

#include "realtime_thread/realtime_thread.h"

#define TM_INFINITE   (0)
#define TM_NONBLOCK   ((xnticks_t)-1)

/* Mutex Interface */
int rt_mutex_create(XN_RT_MUTEX *mutex, const char *name) __attribute__((weak));

int rt_mutex_delete(XN_RT_MUTEX *mutex) __attribute__((weak));

int rt_mutex_acquire(XN_RT_MUTEX *mutex, RTIME timeout) __attribute__((weak));

int rt_mutex_release(XN_RT_MUTEX *mutex) __attribute__((weak));

/* Condition Variable Interface */
int rt_cond_create(XN_RT_COND *cond, const char *name) __attribute__((weak));

int rt_cond_delete(XN_RT_COND *cond) __attribute__((weak));

int rt_cond_signal(XN_RT_COND *cond) __attribute__((weak));

int rt_cond_wait(XN_RT_COND *cond,
		             XN_RT_MUTEX *mutex,
		             RTIME timeout) __attribute__((weak));

/* Task Interface */
int rt_task_shadow(XN_RT_TASK *task,
		               const char *name,
		               int prio,
		               int mode) __attribute__((weak));

#endif


