#!/usr/bin/python

import rostools
rostools.update_path('vslam')
import rostest
import rospy

import pickle
import pprint
import operator

from numpy import array
import numpy
import transformations
from transf_fit import *
# from skeleton import Skeleton

# load pickled files
#trajs_filename = 'trajs.pkl'
trajs_filename = 'keyframe_trajs.pkl'
gt_filename    = 'gt.pkl'
print "loading ",  trajs_filename, "and", gt_filename
trajs_file = open(trajs_filename, 'rb')
gt_file    = open(gt_filename,    'rb')

trajectory = pickle.load(trajs_file)
gt         = pickle.load(gt_file)
# pprint.pprint(trajectory)

trajs_file.close()

print "loaded ", len(trajectory), "trajector(y|ies)"

# loading skeletons
skeleton_nodes_filename = 'skeleton_nodes.pkl'
skeleton_edges_filename = 'skeleton_edges.pkl'

draw_skeleton = True
if draw_skeleton:
  skeleton_nodes_file = open(skeleton_nodes_filename,'rb')
  skeleton_edges_file = open(skeleton_edges_filename,'rb')
  skeleton_nodes = pickle.load(skeleton_nodes_file)
  skeleton_nodes = sorted(skeleton_nodes, key=operator.itemgetter(0))
  skeleton_edges = pickle.load(skeleton_edges_file)


# for i in range(len(trajectory)):
#   for x, y, z in trajectory[i]:
#     print x, y, z

# The numpy array data.
st_trj_points = array(trajectory[0])
trj_numpts = len(trajectory[0])

st_gt_points  = array(gt)
gt_numpts = len(gt)

st_skel_nodes = array(skeleton_nodes)

time_stamp_offset  = st_gt_points[0][0]-st_skel_nodes[0][0]
time_stamp_offset2 = st_gt_points[gt_numpts-1][0]-st_trj_points[trj_numpts-1][0]

st_skel_nodes[0][0] = st_gt_points[0][0] # adjust the time stamp of the first noode since it is not moving anyway.
print 'timestamps', 'phase space:', st_gt_points[0][0], 'vo:', st_trj_points[0][0],
print 'offset:', time_stamp_offset, 'offset2:', time_stamp_offset2
time_stamp_offset = -.00375  # 0.0316576975223 for bag4
time_stamp_offset = -.0038   # 0.0316575769793 for bag4
time_stamp_offset = -.003825 # 0.0316575166683 for bag4
time_stamp_offset = 0.0
print 'use offset', time_stamp_offset
# go thru both trajectories in time, pick matching points from st_gt_points that
# matches st_trj_points the best in time stamps
# st_gt_points_matched = match_trajectory_points(st_trj_points, st_gt_points, time_stamp_offset)
st_gt_points_matched = match_trajectory_points(st_skel_nodes, st_gt_points, time_stamp_offset)
print 'len of st_gt_points_matched', len(st_gt_points_matched),
print 'len of st_trj_points', len(st_trj_points)
print 'len of st_skel_nodes', len(st_skel_nodes)
print 'num of skeleton edges', len(skeleton_edges)/2

#fitting for transformation
shift0 = [0., 0., 0.]
euler0 = [0., 0., 0.]
scale0 = [1.0, 1.0, 1.0]
time_stamp_offset0 = 0.00
transf0=[shift0[0], shift0[1], shift0[2], euler0[0], euler0[1], euler0[2], scale0, time_stamp_offset0]

# transf_fit = transf_fit_space_scale_time
# transf_fit = transf_fit_space
# transf_fit = transf_fit_angle_scale_time
# transf_fit = transf_fit_angle
transf_fit = transf_fit_angle_scale3
transf00 = euler0 + scale0
#transf_fit = transf_fit_angle_scale
#transf00 = euler0 + [1.0]

# transf = fmin_powell(transf_fit, transf00, args=(st_gt_points_matched, st_trj_points))
# print 'fmin_powell, transf', transf

#transf = fmin_cg(transf_fit, transf00, fprime=None, args=(st_gt_points_matched, st_trj_points))
transf = fmin_cg(transf_fit, transf00, fprime=None, args=(st_gt_points_matched, st_skel_nodes))
print 'fmin_cg, transf', transf

#transf = [0.,0.,0.,.975, .975, .975]
#transf[3:6] = [.975, .975, .975]

# translation vector
# p = transf[0:3]
p = [0.,0.,0.]
# euler angles
# e = transf[3:6]
e = transf[0:3]
s = transf[3:6]
# s = [transf[3], transf[3], transf[3]]
  
pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

#st_gt_points = transf_curve(st_gt_points, pose)
#st_gt_points = scale_curve(st_gt_points, s)
st_gt_points = st_gt_points_matched

if 1:
  st_gt_points = transf_curve(st_gt_points, pose)
  st_gt_points = scale_curve3(st_gt_points, s)
  st_gt_points_matched = transf_curve(st_gt_points_matched, pose)
  st_gt_points_matched = scale_curve3(st_gt_points_matched, s)
gt_numpts = len(st_gt_points)

def root_mean_squared_error(curve0, curve1):
  length = len(curve0)
  errors = curve0 - curve1
  rmse = sqrt((errors**2).sum()/length)
  return rmse

transf_ident = [0.,0.,0.,1.,1.,1.]

print 'num of key frames and vo nodes:', len(st_skel_nodes), len(st_trj_points)
#error_sq_skel = transf_fit(transf_ident, st_gt_points_matched, st_skel_nodes)
#error_sq_traj = transf_fit(transf_ident, st_gt_points_matched, st_trj_points)
print 'sqrt of mean of sq of errors (RMSE) of all vslam nodes  ', root_mean_squared_error(st_gt_points_matched, st_skel_nodes)
print 'sqrt of mean of sq of errors (RMSE) of all vo key frames', root_mean_squared_error(st_gt_points_matched, st_trj_points)

dt, dx, dy, dz = st_gt_points_matched[-1]-st_skel_nodes[-1]
error_endpoint = sqrt(dx*dx+dy*dy+dz*dz)
print 'L-2 norm of errors at end point (vslam node vs groundtruth)', error_endpoint
dt, dx, dy, dz = st_gt_points_matched[-1]-st_trj_points[-1]
error_endpoint = sqrt(dx*dx+dy*dy+dz*dz)
print 'L-2 norm of errors at end point (vo end point vs groundtruth)', error_endpoint

# compute the length of trajectory
x0,y0,z0=0.,0.,0.
path_len=0.
for t,x,y,z in st_gt_points:
  path_len += sqrt((x-x0)**2+(y-y0)**2+(z-z0)**2)
  x0,y0,z0 = x,y,z
  
print 'total path length:', path_len, 'meter'

x0,y0,z0=0.,0.,0.
path_len=0.
for t,x,y,z in st_gt_points_matched:
  path_len += sqrt((x-x0)**2+(y-y0)**2+(z-z0)**2)
  x0,y0,z0 = x,y,z
  
print 'total path length (from key frames):', path_len, 'meter'

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



from enthought.tvtk.api import tvtk
from enthought.mayavi.scripts import mayavi2

# The TVTK dataset.
trj_curve = tvtk.PolyData(points=trj_points, lines=trj_lines)
# assign temperature in sequence so that we now the "direction" of the curve
#trj_curve.point_data.scalars = numpy.arange(0,trj_numpts,1)
#trj_curve.point_data.scalars.name = 'temperature'


gt_curve = tvtk.PolyData(points=gt_points, lines=gt_lines)
# assign temperature in sequence so that we now the "direction" of the curve
#gt_curve.point_data.scalars = numpy.arange(0, gt_numpts, 1)
#gt_curve.point_data.scalars.name = 'temperature'

if draw_skeleton:
  points = st_skel_nodes[:,1:4]
  numpts = len(points)
  verts=numpy.arange(0, numpts)
  verts.shape=(numpts, 1)
  skel_nodes_pts = tvtk.PolyData(points=points, verts=verts)
  #skel_nodes_pts.point_data.scalars = numpy.arange(0, numpts, 1)
  #skel_nodes_pts.point_data.scalars.name = 'scalars'
  
  # connect the nodes in time stamp order
  col0 = numpy.arange(0, numpts-1, 1)
  col1 = numpy.arange(1, numpts,   1)
  lines=array([col0, col1]).transpose()
  skel_curve = tvtk.PolyData(points=points, lines=lines)
  
  print "Number of Skeleton Nodes:", numpts
  
  points = array(skeleton_edges)
  num_edges = len(skeleton_edges)
  points.shape = (num_edges*2, 3)
  col0 = numpy.arange(0, num_edges-1, 2)
  col1 = numpy.arange(1, num_edges,   2)
  lines=array([col0, col1]).transpose()
  skel_edges = tvtk.PolyData(points=points, lines=lines)
  #skel_edges.point_data.scalars = numpy.arange(0, num_edges, 1)
  #skel_edges.point_data.scalars.name = 'scalars'

# Uncomment the next two lines to save the dataset to a VTK XML file.
#w = tvtk.XMLPolyDataWriter(input=mesh, file_name='polydata.vtp')
#w.write()

# Now view the data.
@mayavi2.standalone
def view():
  from enthought.mayavi.sources.vtk_data_source import VTKDataSource
  #from enthought.mayavi.modules.outline import Outline
  from enthought.mayavi.modules.surface import Surface
  from enthought.mayavi.modules.axes import Axes
  from enthought.mayavi.modules.text import Text
    
  scene = mayavi.new_scene()
  scene.scene.background=(1.0,1.0,1.0)
  scene.scene.foreground=(0.0,0.0,0.0)
  trj_src = VTKDataSource(data = trj_curve)
  mayavi.add_source(trj_src)
  # mayavi.add_module(Outline())
  s = Surface()
  mayavi.add_module(s)
  s.actor.property.set(representation = 'w', color=(1.,0.,0.), line_width=1)

  gt_src = VTKDataSource(data = gt_curve)
  mayavi.add_source(gt_src)
  #mayavi.add_module(Outline())
  s = Surface()
  mayavi.add_module(s)
  s.actor.property.set(representation = 'w', color=(0.,1.,0.), line_width=2)
    
  # skeleton nodes
  if draw_skeleton:
    # draw the nodes
    skel_node_src = VTKDataSource(data = skel_nodes_pts)
    mayavi.add_source(skel_node_src)
    #mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)
    s.actor.property.set(representation = 'p', point_size = 4, color=(0.,0.,1.0))
      
    # draw edges between nodes
    skel_curve_src = VTKDataSource(data = skel_curve)
    mayavi.add_source(skel_curve_src)
    #mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)
    s.actor.property.set(representation = 'w', color=(1.,0.,1.0), line_width=2)
      
    # draw the links
    skel_edge_src = VTKDataSource(data = skel_edges)
    mayavi.add_source(skel_edge_src)
    #mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)
    s.actor.property.set(representation = 'w', color=(0.,0.,1.), line_width=1)
    
  a = Axes()
  mayavi.add_module(a)
  # Put up some text.
  t = Text(text='VSLAM vs PhaseSpace', x_position=0.2, y_position=0.9, width=0.8)
  t.property.color = 1, 1, 0  # Bright yellow, yeah!
  mayavi.add_module(t)

if __name__ == '__main__':
  view()
