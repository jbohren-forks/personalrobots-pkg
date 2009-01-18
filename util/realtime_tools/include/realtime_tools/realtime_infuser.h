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
/*
  The RealtimeInfuser allows a non-realtime thread to write to a value
  for the realtime thread to read.

  No more than 2 threads are supported: one calling get and another
  calling set.

  Author: Stuart Glaser
 */
#ifndef REALTIME_INFUSER_H
#define REALTIME_INFUSER_H

namespace realtime_tools {

template <class T>
class RealtimeInfuser
{
public:
  RealtimeInfuser(const T &initial = T()) : in_use(0), ready(1)
  {
    set(initial);
  }

  // Called from non-realtime.
  void set(const T &value)
  {
    int next_slot = unused();
    data[next_slot] = value;
    ready = next_slot;
  }

  // True iff new data is ready.
  bool has_next()
  {
    return in_use != ready;
  }

  // Called from realtime.  Returns the next available value.
  T& next()
  {
    in_use = ready;
    return data[in_use];
  }

private:
  // Three slots are used to pass data from non-realtime to realtime.
  // The object in the in_use slot is currently being used by the
  // realtime loop.  The object in the ready slot has been written by
  // the non-realtime.  The last slot is considered unused and is the
  // next place to which the non-realtime thread will write.
  T data[3];
  int in_use, ready;

  // Returns the slot in data that is not being used.
  int unused()
  {
    if (0 != in_use && 0 != ready)
      return 0;
    if (1 != in_use && 1 != ready)
      return 1;
    if (2 != in_use && 2 != ready)
      return 2;
    abort();
  }

};

}

#endif
