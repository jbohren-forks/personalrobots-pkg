#!/usr/bin/env python
import roslib; roslib.load_manifest('outlet_detection')

import numpy
from numpy.linalg import norm
from pylab import load
import sys

data = load(sys.argv[1])
readings = load(sys.argv[2])
pos = [ numpy.array(data[i, :3]) for i in range(readings.size) ]
distances = numpy.array( [1000 * norm(pos[0] - p) for p in pos] )
err = numpy.abs(distances - readings)
print err
print 'Mean:', err.mean()
print 'Std-dev:', err.std()
