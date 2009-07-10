import roslib
roslib.load_manifest('tf')
import rostest
import rospy
import numpy
import rostest
import unittest

import tf.transformations
import robot_msgs.msg

import _tfX

class TestDirected(unittest.TestCase):

    def setUp(self):
        pass

    def expect_exception(self, func, exception):
       tripped = False
       try:
           func()
       except exception:
           tripped = True
       self.assert_(tripped)

    class SubClass(_tfX.Transformer):
        pass

    def test_smoke(self):
        t = _tfX.Transformer(True, rospy.Duration(10.0))
        self.common(t)

    def test_subclass(self):
        class TransformerSubclass(_tfX.Transformer):
            def extra(self):
              return 77
        t = TransformerSubclass(True, rospy.Duration(10.0))
        self.common(t)
        self.assert_(t.extra() == 77)

    def common(self, t):
        m = robot_msgs.msg.TransformStamped()
        m.header.frame_id = "THISFRAME"
        m.parent_id = "PARENT"
        m.transform.translation.y = 5.0
        m.transform.rotation = robot_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        t.setTransform(m)
        afs = t.allFramesAsString()
        self.assert_(len(afs) != 0)
        self.assert_("PARENT" in afs)
        self.assert_("THISFRAME" in afs)
        self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_seconds() == 0)
        for ti in [3, 5, 10, 11, 19, 20, 21]:
            m.header.stamp.secs = ti
            t.setTransform(m)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_seconds() == ti)

        # Verify that getLatestCommonTime with nonexistent frames raises a _tfX.error
        self.expect_exception(lambda: t.getLatestCommonTime("XXX", "YYY"), _tfX.error)

if __name__ == '__main__':
    if 0:
        rostest.unitrun('tf', 'directed', TestDirected)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_smoke'))
        unittest.TextTestRunner(verbosity=2).run(suite)
