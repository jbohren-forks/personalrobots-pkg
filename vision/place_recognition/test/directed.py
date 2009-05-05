import roslib
roslib.load_manifest('place_recognition')
import rostest
import pylab, numpy
import Image
import starfeature
import fast
import calonder

import sys
sys.path.append('lib')
import place_recognition

#ims = [ Image.open("/u/jamesb/ros2/ros-pkg/vision/visual_odometry/key_%06d.png" % (3 * i)) for i in range(1000)]
#ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(100)]

ims = [ Image.open("/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff" % (20 * i)) for i in range(10)]

if 0:
  vt = place_recognition.vocabularytree()
  vt.build(ims, 5, 4, False)
  vt.save("foo")
if 1:
  print "loading..."
  #vt = place_recognition.load("foo")
  vt = place_recognition.load("/u/mihelich/images/holidays/holidays.tree")
  cl = calonder.classifier()
  cl.read('/u/prdata/calonder_trees/current.rtc')

  def kp_d(frame):
    #sd = starfeature.star_detector(frame.size[0], frame.size[1], 5, 10.0, 10.0)
    #kp = [ (x,y) for (x,y,s,r) in sd.detect(frame.tostring()) ]
    fkp = fast.fast(frame.tostring(), frame.size[0], frame.size[1], 15, 9)
    fkp = [ (x,y,r) for (x,y,r) in fast.nonmax(fkp) if (16 <= x and 16 <= y and x <= frame.size[0]-16 and y <= frame.size[1]-16) ]
    kp = [ (x,y) for (x,y,r) in sorted(fkp, key = lambda x:(x[2],x[0],x[1]), reverse = True)[:500] ]
    print 'kp_d gave %d points' % len(kp)

    descriptors = []
    for (x,y) in kp:
      patch = frame.crop((x-16,y-16,x+16,y+16))
      sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
      descriptors.append(sig)
    return descriptors

  print "adding..."
  for i in ims:
    #vt.add(i)
    vt.add(i, kp_d(i))

  print "Do query"
  #M = [ vt.topN(i, None, 10) for i in ims[:100] ]
  M = [ vt.topN(i, kp_d(i), 10) for i in ims[:100] ]

  pylab.pcolor(numpy.array(M))
  pylab.colorbar()
  pylab.show()
