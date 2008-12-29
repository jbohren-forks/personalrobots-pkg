#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Melonee Wise

PKG = "filter_demo"
import rostools; rostools.update_path(PKG)
import numpy
import matplotlib
from pylab import *
import matplotlib.pyplot as plot

in0=[]
in1=[]
out0=[]
out1=[]

for line in  open("output.txt").readlines():
    obj = eval(line)
    in0.append(obj[0])
    out0.append(obj[1])
    in1.append(obj[2])
    out1.append(obj[3])  

fig=plot.figure(1)
axes1 = fig.add_subplot(211)
axes2 = fig.add_subplot(212)
axes1.set_title('Channel 1: sin(2*pi*t/10)')

axes1.set_ylabel('Amplitude')
axes2.set_title('Channel 2: cos(2*pi*t/20)')
axes2.set_xlabel('N')
axes2.set_ylabel('Amplitude')

#plot the effort hysteresis
axes1.plot(numpy.array(in0), 'b', label='input')
axes1.plot(numpy.array(out0), 'r', label='filtered output')
axes1.legend()
#show the average effort lines 
axes2.plot(numpy.array(in1), 'b', label='input')
axes2.plot(numpy.array(out1), 'r', label='filtered output')
axes2.legend()
show()
