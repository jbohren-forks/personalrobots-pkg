import rostools
rostools.update_path('vslam')
import rostest
import rospy

import Image

import camera
import pylab, numpy

from stereo import ComputedDenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

vo = VisualOdometer(stereo_cam, feature_detector = FeatureDetectorStar(), descriptor_scheme = DescriptorSchemeCalonder())

(f0,f1) = [ SparseStereoFrame(Image.open("f%d-left.png" % i), Image.open("f%d-right.png" % i)) for i in [0, 1] ]

vo.setup_frame(f0)
vo.setup_frame(f1)

pairs = vo.temporal_match(f0, f1)
for (a,b) in pairs:
  pylab.plot([ f0.kp[a][0], f1.kp[b][0] ], [ f0.kp[a][1], f1.kp[b][1] ])
pylab.imshow(numpy.fromstring(f0.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
pylab.scatter([x for (x,y,d) in f0.kp], [y for (x,y,d) in f0.kp], label = 'f0 kp', c = 'red')
pylab.scatter([x for (x,y,d) in f1.kp], [y for (x,y,d) in f1.kp], label = 'f1 kp', c = 'green')
pylab.legend()
pylab.show()
