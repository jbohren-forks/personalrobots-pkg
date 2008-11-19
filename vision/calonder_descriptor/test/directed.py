import rostools
rostools.update_path('calonder_descriptor')
import rostest

import sys
sys.path.append('lib')
import calonder
import Image
import ImageChops
import ImageDraw

import unittest
import random
import time
import math

import fast

class TestDirected(unittest.TestCase):

    def setUp(self):
      pass

    def test_identity(self):
      im = Image.open("/u/konolige/vslam/data/indoor1/left-%04d.ppm" % 1000)
      kp = [(x-16, y-16) for (x,y) in fast.fast(im.tostring(), im.size[0], im.size[1], 150, 40)]

      cl1 = calonder.classifier()
      cl1.setThreshold(0.0)

      cl1.train(im.tostring(), im.size[0], im.size[1], kp)
      cl1.write('unittest.tree')

      def testclassifier(kp, im, cl):
        ma = calonder.BruteForceMatcher()

        sigs = []
        for (x,y) in kp:
          patch = im.crop((x,y,x+32,y+32))
          sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
          #print sig.dump()
          sigs.append(sig)
          ma.addSignature(sig)

        for (i,(x,y)) in enumerate(kp):
          patch = im.crop((x,y,x+32,y+32))
          sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
          (index, distance) = ma.findMatch(sig)
          self.assert_(i == index)

      testclassifier(kp, im, cl1)
      del cl1

      # Now make another classifier, and read it from the file above

      cl2 = calonder.classifier()
      cl2.setThreshold(0.0)
      cl2.read('unittest.tree')

      testclassifier(kp, im, cl2)


if __name__ == '__main__':
    #rostest.unitrun('calonder_descriptor', 'directed', TestDirected)
    if 1:
      suite = unittest.TestSuite()
      suite.addTest(TestDirected('test_identity'))
      unittest.TextTestRunner(verbosity=2).run(suite)
