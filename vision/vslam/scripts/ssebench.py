#!/usr/bin/env python

import roslib
roslib.load_manifest('vslam')
import rostest
import rospy

import time
import vop

vec_length = 5000

a = vop.arange(vec_length)
b = vop.arange(vec_length)
c = vop.arange(vec_length)

# Warm up
x = vop.sqrt(a * a + b * b + c * c)

start_t = time.time()
start_o = vop.odometer()

for i in range(1000000):
    x = vop.sqrt(a * a + b * b + c * c)

end_t = time.time()
end_o = vop.odometer()

t = end_t - start_t
o = end_o - start_o
print "%d operations took %fs, %f megaflops" % (o, t, (1.e-6 * o / t))

print vop.lerp2(vop.arange(10) / 10, vop.arange(10), vop.arange(10)).tolist()
print vop.lerp2(0.5, vop.arange(10), vop.arange(10)).tolist()
print vop.lerp2(0.5, vop.arange(10), 10.).tolist()
