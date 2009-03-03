import roslib
roslib.load_manifest('pytoro')
import rostest
import rospy

import unittest
import math

from pytoro import TreeOptimizer3

def mk_covar(xyz, rp, yaw):
  return (1.0 / math.sqrt(xyz),1.0 / math.sqrt(xyz), 1.0 / math.sqrt(xyz), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(rp), 1.0 / math.sqrt(yaw))
#weak = mk_covar(0.01, 0.0002, 0.002)
weak = mk_covar(9e10,3,3)
strong = mk_covar(0.0001, 0.000002, 0.00002)

class TestDirected(unittest.TestCase):

  """ Empty graph operations """
  def test_null(self):
    g = TreeOptimizer3()
    g.initializeOnlineOptimization()
    g.iterate()

  def test_simple(self):

    # Consistent graph has zero error
    g = TreeOptimizer3()
    g.initializeOnlineOptimization()
    g.addIncrementalEdge(0, 1, (1, 0, 0), (0, 0, 0))
    g.addIncrementalEdge(1, 0, (-1, 0, 0), (0, 0, 0))
    self.assert_(g.error() == 0.0)
    print g.vertex(0)
    print g.vertex(1)

    # Save and load the consistent graph
    g.save("test1.toro")

    g1 = TreeOptimizer3()
    g1.load("test1.toro")
    print g1.vertex(0)
    print g1.vertex(1)
    g1.addIncrementalEdge(0,2, (1,0,0), (0,0,0))
    g1.addIncrementalEdge(1,2, (1,0,0), (0,0,0))
    g1.initializeOnlineOptimization()

if __name__ == '__main__':
  if 0:
    rostest.unitrun('pytoro', 'directed', TestDirected)
  else:
    suite = unittest.TestSuite()
    #suite.addTest(TestDirected('test_simple'))
    unittest.TextTestRunner(verbosity=2).run(suite)
