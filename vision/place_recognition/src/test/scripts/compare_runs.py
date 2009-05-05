#! /usr/bin/python
import roslib
roslib.load_manifest('place_recognition')
import rostest
import pylab, numpy
import Image
import fast
import calonder
import operator

import sys
sys.path.append('lib')
import place_recognition

def get_features(frame, classifier):
  # Get keypoints from FAST detector
  fkp = fast.fast(frame.tostring(), frame.size[0], frame.size[1], 12, 9)
  # Keep local maxima that are at 16 pixels away from image border
  fkp = [ (x,y,r) for (x,y,r) in fast.nonmax(fkp) if (16 <= x and 16 <= y and x <= frame.size[0]-16 and y <= frame.size[1]-16) ]
  # Keep at most 500 strongest keypoints
  kp = [ (x,y) for (x,y,r) in sorted(fkp, key = lambda x:(x[2],x[0],x[1]), reverse = True)[:500] ]

  # Compute Calonder descriptor for each keypoint
  descriptors = []
  for (x,y) in kp:
    patch = frame.crop((x-16,y-16,x+16,y+16))
    sig = classifier.getSignature(patch.tostring(), patch.size[0], patch.size[1])
    descriptors.append(sig)
  return descriptors, kp

def match_runs(reference_files, query_files):
  print "loading vocabulary tree..."
  vt = place_recognition.load("/u/mihelich/images/holidays/holidays.tree")
  cl = calonder.classifier()
  cl.read('/u/prdata/calonder_trees/current.rtc')

  print "adding database images..."
  for i in reference_files[2]:
    # Add image to vocabulary tree
    name = reference_files[0] % i
    img = Image.open(name)
    desc, kp = get_features(img, cl)
    vt.add(img, desc)
    print '%s has %d feature points' % (name, len(kp))
    # Save coordinates and descriptors
    file = open(reference_files[1] % (i, "key"), "w")
    for (pt, d) in zip(kp, desc):
      # Format: x y [d...]
      file.write("%i %i %s\n" % (pt[0], pt[1], str(list(d))))

  print "getting top-k views for query images..."
  N = len(reference_files[2])
  k = 32
  M = []
  for i in query_files[2]:
    # Get indexes of best N matching views from reference run
    name = query_files[0] % i
    img = Image.open(name)
    desc, kp = get_features(img, cl)
    print '%s has %d feature points' % (name, len(kp))
    scores = vt.topN(img, desc, N)
    M.append(scores)
    top_n = sorted(enumerate(scores), key=operator.itemgetter(1))[:k]
    # Save filenames of matching views
    file = open(query_files[1] % (i, "match"), "w")
    for (index, score) in top_n:
      file.write("%s %f\n" % (reference_files[0] % reference_files[2][index], score))
  return M

def plot_confusion(M):
  # Show full score matrix
  pylab.pcolor(numpy.array(M))
  pylab.colorbar()
  pylab.show()

# Tuples describing file names (image format, output format, range)
run1_files = ("/u/kosecka/research/run1/%06uL.png", "run1_%06uL.%s", range(2400, 3401))
run2_files = ("/u/kosecka/research/run2/%06uL.png", "run2_%06uL.%s", range(2200, 3201))

# run 1 in vocabulary tree, run 2 used as queries
M = match_runs(run1_files, run2_files)
pylab.save('run1vt_run2q.mat', M)
#plot_confusion(M)

# run 2 in vocabulary tree, run 1 used as queries
M = match_runs(run2_files, run1_files)
pylab.save('run2vt_run1q.mat', M)
#plot_confusion(M)
