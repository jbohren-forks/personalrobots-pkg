#!/usr/bin/python -i 
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Written by Timothy Hunter <tjhunter@willowgarage.com> 2008     
     
import pickle
     
__all__ = ['smooth_data', 'decimate_data', 'timify', 'min_stamp', 'run_time', 'smooth','load_traj','dump_traj']        
def smooth_data(data, smoothing_factor=4):
  res=[]
  for j in range(len(data)-1):
    print j
    (t1,l1)=data[j]
    (t2,l2)=data[j+1]
    SMOOTH=smoothing_factor
    for i in range(SMOOTH):
      l=[]
      foo=i*1.0/SMOOTH
      for j in range(len(l1)):
        c=l1[j][0]
        d1=l1[j][1]
        d2=l2[j][1]
        l.append((c,(1-foo)*d1+foo*d2))
      res.append((t1/SMOOTH,l))
  return res      

def decimate_data(data, decimation_factor=2):
  res=[]
  for j in range(len(data)):
    if j % decimation_factor == 1:
      res.append(data[j])
  return res

def timify(data, factor=1.0):
  return [(factor*t,l) for (t,l) in data]

def min_stamp(data):
  return min([t for (t,l) in data[1:]])

def run_time(data):
  return sum([t for (t,l) in data])

def smooth(data, run_time_=None):
  FRAME_TIME = 0.0011
  '''Constraints: timestamps > 1/1000'''
  rt = run_time(data)
  # Time dilatation factor
  if run_time_:
    t_factor=run_time_/rt
  else:
    run_time_=rt
    t_factor=1.0
  print 't_factor',t_factor
  ms=min_stamp(data)
  d_factor = int(FRAME_TIME/(ms*t_factor))
  if d_factor > 1:
    print 'd_factor',d_factor
    f=decimate_data(data,d_factor)
    return timify(f,run_time_/run_time(f))
  s_factor=int(ms*t_factor/FRAME_TIME)
  if s_factor > 1:
    print 's_factor',s_factor
    f=smooth_data(data, s_factor)
    return timify(f,run_time_/run_time(f))
  return timify(data,t_factor)

def load_traj(fname):
  f=open(fname,'rb')
  return pickle.load(f)

def dump_traj(object, fname):
  f=open(fname,'wb')
  pickle.dump(object,f)