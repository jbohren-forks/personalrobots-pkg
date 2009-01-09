import pickle
import pprint

# load pickled files
trajs_file = open('trajs.pkl', 'rb')
gt_file    = open('gt.pkl',    'rb')

trajectory = pickle.load(trajs_file)
gt         = pickle.load(gt_file)
#pprint.pprint(trajectory)

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

# The numpy array data.
points = array(trajectory[0])
numpts = len(trajectory[0])

points = array(gt)
numpts = len(gt)

col0 = numpy.arange(0,numpts-1,1)
col1 = numpy.arange(1,numpts,1)
lines=array([col0, col1]).transpose()


# The TVTK dataset.
mesh = tvtk.PolyData(points=points, lines=lines)
mesh.point_data.scalars = numpy.ones(numpts)
mesh.point_data.scalars.name = 'scalars'

# Uncomment the next two lines to save the dataset to a VTK XML file.
#w = tvtk.XMLPolyDataWriter(input=mesh, file_name='polydata.vtp')
#w.write()

# Now view the data.
def view():
    from enthought.mayavi.sources.vtk_data_source import VTKDataSource
    from enthought.mayavi.modules.outline import Outline
    from enthought.mayavi.modules.surface import Surface

    mayavi.new_scene()
    src = VTKDataSource(data = mesh)
    mayavi.add_source(src)
    mayavi.add_module(Outline())
    s = Surface()
    mayavi.add_module(s)

view()
