import roslib; roslib.load_manifest('bullet')

import bullet

import sys
import unittest
import math


## A sample python unit test
class AngleConversions(unittest.TestCase):
    def setUp(self):
        #Setup the pose tests
        #self.quaternion_from_euler = bullet.Quaternion(0,0,0)
        pass
    
    # Test Euler conversions
    def test_euler_quaternion_euler(self):
        yprs = [(0,0,0),
                #\todo normalize on +-pi (math.pi*3/2, 0, 0),
                (math.pi, 0, 0),
                (math.pi/2, 0, 0),
                (math.pi/4, 0, 0),
                (math.pi/8, 0, 0),
                (math.pi/2, math.pi/8, 0),
                (math.pi/4, math.pi/8, 0),
                (math.pi/8, math.pi/8, 0),
                (math.pi/2, 0, math.pi/8),
                (math.pi/4, 0, math.pi/8),
                (math.pi/8, 0, math.pi/8),
                (math.pi/2, math.pi/8, math.pi/8),
                (math.pi/4, math.pi/8, math.pi/8),
                (math.pi/8, math.pi/8, math.pi/8),
                (math.pi/8, 0, 0),
                (math.pi/4, 0, 0),
                (math.pi/2, 0, 0)]
        for eulers in yprs:
            print eulers
            quat = bullet.Quaternion(eulers[0],eulers[1],eulers[2])
            yaw = bullet.Matrix3x3(quat).getEulerZYXYaw()
            self.assertAlmostEqual(yaw, eulers[0] , 7, "yaw %f euler to quaternion to euler %f correctness"%(yaw, eulers[0]))
            
            # \todo deal with redundancy
            #pitch = bullet.Matrix3x3(quat).getEulerZYXPitch(0)
            #self.assertAlmostEqual(pitch, eulers[0] , 7, "pitch %f euler to quaternion to euler %f correctness"%(pitch, eulers[0]))
            #roll = bullet.Matrix3x3(quat).getEulerZYXRoll(1)
            #self.assertAlmostEqual(roll, eulers[0] , 7, "roll %f euler to quaternion to euler %f correctness"%(roll, eulers[0]))

    # Test Euler conversions
    def test_quaternion_euler(self):
        ground_truth = [((0,0,0,1), (0,0,0)),
                        ((0,0,1,0), (math.pi,0,0)),
                        ((0.000000, 0.000000, math.sqrt(2)/2, math.sqrt(2)/2), (math.pi/2,0,0)),
                        ((0.000000, 0.000000, 0.382683, 0.923880), (math.pi/4,0,0)),#\todo had to decrease accuracy due to cut and paste number, should use analytic like above
                        ]

        for quattuple, eulers in ground_truth:
            print quattuple
            quat = bullet.Quaternion(quattuple[0],quattuple[1],quattuple[2],quattuple[3])
            yaw = bullet.Matrix3x3(quat).getEulerZYXYaw()
            self.assertAlmostEqual(yaw, eulers[0] , 5, "yaw %f quaternion to euler %f correctness"%(yaw, eulers[0])) 
            
            # \todo deal with redundancy
            #pitch = bullet.Matrix3x3(quat).getEulerZYXPitch(0)
            #self.assertAlmostEqual(pitch, eulers[0] , 5, "pitch %f euler to quaternion to euler %f correctness"%(pitch, eulers[0]))
            #roll = bullet.Matrix3x3(quat).getEulerZYXRoll(1)
            #self.assertAlmostEqual(roll, eulers[0] , 5, "roll %f euler to quaternion to euler %f correctness"%(roll, eulers[0]))


if __name__ == '__main__':
    import rostest
    rostest.unitrun('bullet', 'test_angle_conversions', AngleConversions)
                            
