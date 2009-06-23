import roslib
roslib.load_manifest('outlet_detection')
import rostest
import pylab, numpy
from operator import add
import Image
import fast
import calonder
import place_recognition

outlet_files = '/wg/wgdata1/vol1/Outlets/OutletWhiteHall_cropped_outlets/IMG_%04d.JPG'
no_out_files = '/wg/wgdata1/vol1/Outlets/OutletWhiteHall_no_outlet/IMG_%04d.JPG'

print 'opening images...'
outlet_train = [ Image.open(outlet_files % i).convert('L') for i in range(416, 427) ]
no_out_train = [ Image.open(no_out_files % i).convert('L') for i in range(416, 427) ]
#outlet_test  = [ Image.open(outlet_files % i).convert('L') for i in range(372, 416) + range(427, 471) ]
#no_out_test  = [ Image.open(no_out_files % i).convert('L') for i in range(372, 416) + range(427, 471) ]

print 'loading trees...'
vt = place_recognition.load('/u/mihelich/images/holidays/holidays_b5d4.tree')
#vt = place_recognition.load('/u/mihelich/images/holidays/holidays.tree')
#vt = place_recognition.vocabularytree()
#vt.build(outlet_train, 5, 4, False)
cl = calonder.classifier()
cl.read('/u/prdata/calonder_trees/current.rtc')

def kp_d(frame):
  fkp = fast.fast(frame.tostring(), frame.size[0], frame.size[1], 10, 15)
  fkp = [ (x,y,r) for (x,y,r) in fkp if (16 <= x and 16 <= y and x <= frame.size[0]-16 and y <= frame.size[1]-16) ]
  fkp = fast.nonmax(fkp) #damn this is slow
  kp = [ (x,y) for (x,y,r) in sorted(fkp, key = lambda x:(x[2],x[0],x[1]), reverse = True)[:200] ]
  print 'kp_d gave %d points' % len(kp)

  descriptors = []
  for (x,y) in kp:
    patch = frame.crop((x-16,y-16,x+16,y+16))
    sig = cl.getSignature(patch.tostring(), patch.size[0], patch.size[1])
    descriptors.append(sig)
  return descriptors

print 'adding training images...'
#vt.add(0, reduce(add, [kp_d(i) for i in outlet_train]))
#vt.add(1, reduce(add, [kp_d(i) for i in no_out_train]))
for i in outlet_train + no_out_train:
  vt.add(i, kp_d(i))

print 'querying outlet test images...'
M = [ vt.topN(i, kp_d(i), 22) for i in outlet_train ]

pylab.pcolor(numpy.array(M))
pylab.colorbar()
pylab.show()
