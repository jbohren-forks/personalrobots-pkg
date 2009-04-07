import roslib
roslib.load_manifest('calonder_descriptor')
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
import pickle

import fast

class TestDirected(unittest.TestCase):

    def setUp(self):
      pass

    def test_identity(self):
      im = Image.open("f0-left.png")
      kp = [(x-16, y-16) for (x,y,r) in fast.fast(im.tostring(), im.size[0], im.size[1], 55, 0) if (x > 32) and (y > 32) and (x < (640-32)) and (y < (480-32))]
      print "keypoints", len(kp)
      dim = 176 # actual dimension will be min(176, |kp|)

      cl1 = calonder.classifier()

      if 0:
        cl1.train(im.tostring(), im.size[0], im.size[1], kp, 50, 10, 100, dim, 0)
      else:
        # CMakefile.txt downloads current.rtc before this test runs
        filename = 'current.rtc'
        cl1.read(filename)

      dim = cl1.dimension()

      for i in range(1000000):
        print i
        sigs = cl1.getSignatures(im, kp)
        #for (x,y) in kp:
        #  patch = im.crop((x,y,x+32,y+32))
        #  sig = cl1.getSignature(patch.tostring(), patch.size[0], patch.size[1])
      return

      def testclassifier(kp, im, cl):
        ma = calonder.BruteForceMatcher(dim)

        sigs = []

        kp = kp[:]
        for (x,y) in kp:
          print x,y
          patch = im.crop((x,y,x+32,y+32))
          sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
          sigs.append(sig)
          ma.addSignature(sig)
        #print cl.getSignatures(im, kp)

        for (i,(x,y)) in enumerate(kp):
          patch = im.crop((x,y,x+32,y+32))
          sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
          (index, distance) = ma.findMatch(sig)
          print "i = %d, match = %u, distance = %.3f" % (i, index, distance)
          self.assert_(i == index)

      testclassifier(kp, im, cl1)
      print "done"

      print "Writing to unittest.tree... ",
      cl1.write('unittest.tree')
      print "done"

      del cl1

      # Now make another classifier, and read it from the file above

      cl2 = calonder.classifier()
      print "Reading classifier... ",
      cl2.read('unittest.tree')
      print "done"

      testclassifier(kp, im, cl2)

      f = open('sigs.pickle', 'w')
      for (x,y) in kp[:3]:
        patch = im.crop((x,y,x+32,y+32))
        sig = cl2.getSignature(patch.tostring(), patch.size[0], patch.size[1])
        pickle.dump(sig, f)
        print "saved", sig
      f.close()
      f = open('sigs.pickle')
      for i in range(3):
        print "loaded", pickle.load(f)


if __name__ == '__main__':
    #rostest.unitrun('calonder_descriptor', 'directed', TestDirected)
    if 1:
      suite = unittest.TestSuite()
      suite.addTest(TestDirected('test_identity'))
      unittest.TextTestRunner(verbosity=2).run(suite)
