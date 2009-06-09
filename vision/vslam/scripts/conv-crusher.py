#
# converts crusher IMU/SPAN pose files into indexed poes file based on just some views
# first arg is crusher pose file
# second arg is VO pose file
#

# file lines have this form for Crusher data:
# frame: 118016 time: 1188408997.291066 pose: -5831.709908 -344.038363 -9.596183 6.622948 -1.230735 42.080087

# file lines have this form for VO data:
# VERTEX3 66 0.0301717 -0.0222078 0.0402928 0.00618862 0.00948001 0.0132785
# note that these start at 0, we get the offset from the first line of the Crusher data
#

import numpy
import math
import random
import time
import sys


print sys.argv[1]
print sys.argv[2]

fstart = -1

# open crusher pose file
cfile = open(sys.argv[1],'r');

# make dictionary of poses
dcs = {}  # dictionary for IMU poses
for line in cfile:
    a = line.split();
    pose = [float(a[5]), float(a[6]), float(a[7]), float(a[8]), float(a[9]), float(a[10])]
    dcs[int(a[1])] = pose
    if (fstart < 0):
        fstart = int(a[1])
        print "Starting frame is "+str(fstart)


# get just the poses used by VO
cposes = []
vposes = []
vfile = open(sys.argv[2],'r');
for line in vfile:
    a = line.split()
    if (a[0] == "VERTEX3"):
        ind = int(a[1])+fstart          # frame number in crusher data
        if (ind in dcs):
            cposes.append(dcs[ind])
            vposes.append([float(a[2]), float(a[3]), float(a[4]), float(a[5]), float(a[6]), float(a[7])])
            

print "Number of poses: "+str(len(cposes))

ofc = open("ctraj.dat",'w')
ofv = open("vtraj.dat",'w')

for p in cposes:
    ofc.write(str(p[0])+" "+str(p[1])+" "+str(p[2])+" "+str(p[3])+" "+str(p[4])+" "+str(p[5])+"\n")

for p in vposes:
    ofv.write(str(p[0])+" "+str(p[1])+" "+str(p[2])+" "+str(p[3])+" "+str(p[4])+" "+str(p[5])+"\n")


