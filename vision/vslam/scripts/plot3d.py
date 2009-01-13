#!/usr/bin/python

import rostools
rostools.update_path('vslam')
import rostest
import rospy

import pickle
import pprint

# load pickled files
trajs_filename = 'trajs.pkl'
gt_filename    = 'gt.pkl'
print "loading ",  trajs_filename, "and", gt_filename
trajs_file = open(trajs_filename, 'rb')
gt_file    = open(gt_filename,    'rb')

trajectory = pickle.load(trajs_file)
gt         = pickle.load(gt_file)
# pprint.pprint(trajectory)

trajs_file.close()

print "loaded ", len(trajectory), "trajector(y|ies)"

# for i in range(len(trajectory)):
#   for x, y, z in trajectory[i]:
#     print x, y, z

# The following *optional* two lines allow a user to call this script
# as either `python script.py` or `mayavi2 script.py`.  These two
# lines must be placed before any other mayavi imports.
from enthought.mayavi.scripts import mayavi2
mayavi2.standalone(globals())

from numpy import array
from enthought.tvtk.api import tvtk
import numpy
import transformations
from transf_fit import *


# The numpy array data.
st_trj_points = array(trajectory[0])
trj_numpts = len(trajectory[0])

st_gt_points  = array(gt)
gt_numpts = len(gt)

time_stamp_offset  = st_gt_points[0][0]-st_trj_points[0][0]
time_stamp_offset2 = st_gt_points[gt_numpts-1][0]-st_trj_points[trj_numpts-1][0]
print 'timestamps', 'phase space:', st_gt_points[0][0], 'vo:', st_trj_points[0][0],
print 'offset:', time_stamp_offset, 'offset2:', time_stamp_offset2

# go thru both trajectories in time, pick matching points from st_gt_points that
# matches st_trj_points the best in time stamps
st_gt_points_matched = match_trajectory_points(st_trj_points, st_gt_points, time_stamp_offset)
print 'len of st_gt_points_matched', len(st_gt_points_matched),
print 'len of st_trj_points', len(st_trj_points)

#fitting for transformation
shift0 = [0., 0., 0.]
euler0 = [0., 0., 0.]
scale0 = 1.0
time_stamp_offset0 = 0.00
transf0=[shift0[0], shift0[1], shift0[2], euler0[0], euler0[1], euler0[2], scale0, time_stamp_offset0]

# transf_fit = transf_fit_space_scale_time
# transf_fit = transf_fit_space
# transf_fit = transf_fit_angle_scale_time
transf_fit = transf_fit_angle
transf00 = transf0[3:6]

transf = fmin_powell(transf_fit, transf00, args=(st_gt_points_matched, st_trj_points))
print 'fmin_powell, transf', transf

# translation vector
# p = transf[0:3]
p = [0.,0.,0.]
# euler angles
# e = transf[3:6]
e = transf[0:3]
  
pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

st_gt_points = transf_curve(st_gt_points, pose)

if st_trj_points.shape[1]==3:
  # Old format. No time stamp
  trj_points = st_trj_points
else:
  trj_points = st_trj_points[:,1:4]

if st_gt_points.shape[1]==3:
  # Old format. No time stamp
  gt_points = st_gt_points
else:
  gt_points = st_gt_points[:,1:4]
  

col0 = numpy.arange(0,trj_numpts-1,1)
col1 = numpy.arange(1,trj_numpts,1)
trj_lines=array([col0, col1]).transpose()

col0 = numpy.arange(0,gt_numpts-1,1)
col1 = numpy.arange(1,gt_numpts,1)
gt_lines=array([col0, col1]).transpose()


# The TVTK dataset.
trj_curve = tvtk.PolyData(points=trj_points, lines=trj_lines)
# assign temperature in sequence so that we now the "direction" of the curve
trj_curve.point_data.scalars = numpy.arange(0,trj_numpts,1)
trj_curve.point_data.scalars.name = 'temperature'


gt_curve = tvtk.PolyData(points=gt_points, lines=gt_lines)
# assign temperature in sequence so that we now the "direction" of the curve
gt_curve.point_data.scalars = numpy.arange(0, gt_numpts, 1)
gt_curve.point_data.scalars.name = 'temperature'

# Uncomment the next two lines to save the dataset to a VTK XML file.
#w = tvtk.XMLPolyDataWriter(input=mesh, file_name='polydata.vtp')
#w.write()

# Now view the data.
def view():
    from enthought.mayavi.sources.vtk_data_source import VTKDataSource
    from enthought.mayavi.modules.outline import Outline
    from enthought.mayavi.modules.surface import Surface

    mayavi.new_scene()
    trj_src = VTKDataSource(data = trj_curve)
    mayavi.add_source(trj_src)
    mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)

    gt_src = VTKDataSource(data = gt_curve)
    mayavi.add_source(gt_src)
    mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)

view()
