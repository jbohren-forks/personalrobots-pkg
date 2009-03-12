#!/usr/bin/env python
import roslib; roslib.load_manifest('outlet_detection')

import numpy
from pylab import load
import sys
from math import acos, pi

# Load pose data
plug_data = load(sys.argv[1])
outlet_data = load(sys.argv[2])
num_poses = len(plug_data)
assert(num_poses == len(outlet_data))

# Position stats
plug_pos = [ numpy.array(plug_data[i, :3]) for i in range(num_poses) ]
outlet_pos = [ numpy.array(outlet_data[i, :3]) for i in range(num_poses) ]
distances = numpy.array( [ 1000 * numpy.linalg.norm(plug_pos[i] - outlet_pos[i]) for i in range(num_poses) ] )
print 'Distance:\n\tMean = %fmm\n\tStd dev = %f' % (distances.mean(), distances.std())

# Orientation stats
plug_ori = [ numpy.array(plug_data[i, 3:]) for i in range(num_poses) ]
outlet_ori = [ numpy.array(outlet_data[i, 3:]) for i in range(num_poses) ]
angles = numpy.array( [ acos(numpy.dot(plug_ori[i], outlet_ori[i])) * 180.0 / pi for i in range(num_poses) ] )
print 'Angle:\n\tMean = %f degrees\n\tStd dev = %f' % (angles.mean(), angles.std())
