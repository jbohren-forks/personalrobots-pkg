#!/usr/bin/python

"""    Usage: python plot_phsp.py <bag>
"""

import rostools
rostools.update_path('vslam')
import rospy
import sys
import numpy
from numpy import array
import rosrecord
import pickle

filename = sys.argv[1]

stamped_points = []

if 0:
  start,end = 20000,21300
  skipto = 12634364243
  start,end = 0,1300
else:
  skipto = None
  start,end = 0, 40000
  
print "starting loop"
f = open(filename)
frame_counter=0
num_empty_snapshots=0
empty_snapshots=[]
for topic, msg, timeStamp in rosrecord.logplayer(filename):
  #  print 'topic:',topic
  if skipto and (f.tell() < skipto):
    f.seek(skipto)
  #print f.tell(), msg
  if rospy.is_shutdown():
    break

  if topic.endswith("phase_space_snapshot"):
    if frame_counter>=start and frame_counter < end:
      if len(msg.bodies)>0:
        # convert the time stamp to seconds in float
        secs = timeStamp.secs + timeStamp.nsecs*1.e-9
        p = msg.bodies[0].pose.translation
        stamped_points.append((secs, p.x, p.y, p.z))
      else:
        num_empty_snapshots += 1
        empty_snapshots += [timeStamp]
    
    frame_counter += 1
    if frame_counter >= end:
      break

st_points = array(stamped_points)
points = st_points[:,1:4]
numpts = len(points)

#delta_time = array(stamped_points[:numpts][0] - stamped_points[0:numpts-1][0])
delta_time = st_points[1:numpts, 0] - st_points[0:numpts-1, 0]
print 'len of trajectories', len(st_points)
print 'num of empty snapshots', num_empty_snapshots
print 'len of delta_time', len(delta_time)
print 'max delta time', delta_time.max()
print 'min delta time', delta_time.min()
delta_time_mean = delta_time.mean()

print 'mean dela time, in hz', delta_time_mean, 1/delta_time_mean
print 'variance delta time', delta_time.var()

large_time_gaps = [d for d in delta_time if d > 2*delta_time_mean]
print 'num of large time gaps: ', len(large_time_gaps)

for n in [1, 1.5, 2, 3,4,5,6,7]:
  large_gaps = [d for d in delta_time if d > n*delta_time_mean]
  print 'num of large time gaps larger than ',n,' times of mean: ', len(large_gaps)

# Now view the data.
try:
  from enthought.tvtk.api import tvtk
  from enthought.mayavi.scripts import mayavi2
except ImportError:
  print 'please install mayavi2 to visualize the curve:'
  print 'sudo apt-get install mayavi2'
  sys.exit()
  
@mayavi2.standalone
def view():
  col0 = numpy.arange(0,numpts-1,1)
  col1 = numpy.arange(1,numpts,1)
  lines=array([col0, col1]).transpose()
    
  curve = tvtk.PolyData(points=points, lines=lines)
  # assign temperature in sequence so that we now the "direction" of the curve
  curve.point_data.scalars = numpy.arange(0, numpts, 1)
  curve.point_data.scalars.name = 'temperature'

  from enthought.mayavi.sources.vtk_data_source import VTKDataSource
  #from enthought.mayavi.modules.outline import Outline
  from enthought.mayavi.modules.surface import Surface
  from enthought.mayavi.modules.axes import Axes
  from enthought.mayavi.modules.text import Text
    
  scene = mayavi.new_scene()
  scene.scene.background=(1.0,1.0,1.0)
  scene.scene.foreground=(0.0,0.0,0.0)

  src = VTKDataSource(data = curve)
  mayavi.add_source(src)
  #mayavi.add_module(Outline())
  s = Surface()
  mayavi.add_module(s)
  s.actor.property.set(representation = 'p', color=(0.,1.,0.), line_width=2)
    
if __name__ == '__main__':
  view()
