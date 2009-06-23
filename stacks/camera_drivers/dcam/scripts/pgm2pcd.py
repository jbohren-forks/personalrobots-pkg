#
# converts pgm files from Zcam to pcd's with color
# first arg is input pcd file
# second arg is input color file
#

# file lines have this form for PGM depth data:
# P2
# 320 240
# 255
# 0 0 0 0.... (320 times)
# (240 lines)
#


import numpy
import math
import random
import time
import sys


print sys.argv[1]
print sys.argv[2]


# open files
dfile = open(sys.argv[1],'r');
cfile = open(sys.argv[2],'r');
w = 0                                   # size of image
h = 0
n = 0                                   # line number
tot = 0                                 # total number of points

# set up arguments
cutoff = 0
if len(sys.argv) > 3:                   # cut off depth values below this
    cutoff = int(sys.argv[3])

scale = 1.0                             # scale z values by this factor
if len(sys.argv) > 4:
    scale = float(sys.argv[4])


# read in all depth points and convert
pts = []                                # list of points
for line in dfile:
    n += 1
    a = line.split()
    if (n == 2):
        w = int(a[0])
        h = int(a[1])
        print "Size is "+str(w)+"x"+str(h)
        xoff = 0.5
        yoff = 0.5
    if (n > 3):
        j = 0
        for tk in a:
            pt = [float(j)/float(w)-xoff, float(n-1)/float(w)-yoff, float(tk)*scale/400.0]
            if int(tk) > cutoff:
                tot += 1
            else:
                pt[2] = -1.0            # not a valid point
            pts.append(pt)
            j += 1


mx = max([pt[2] for pt in pts])
mn = min([pt[2] for pt in pts])

print mx
print mn
offset = 0.0                            # z offset for standoff

# read in all color points
cls = []                                # list of colors
n = 0
for line in cfile:
    n += 1
    a = line.split()
    if (n == 2):
        w = int(a[0])
        h = int(a[1])
        print "Size is "+str(w)+"x"+str(h)
    if (n > 3):
        for tk in a:
            k = 0
            pt = float(tk)/255.0
            cls.append(pt)


# make pcd file
ofc = open("pts.pcd",'w')

ofc.write("COLUMNS x y z r g b\n")
ofc.write("POINTS "+str(tot)+"\n")
ofc.write("DATA ascii\n")

i = 0
for pt in pts:
    if pt[2] >= 0.0:
        ofc.write(str(pt[0])+" "+str(pt[1])+" "+str(pt[2])+" "+
                  str(cls[i*3])+" "+str(cls[i*3+1])+" "+str(cls[i*3+2])+"\n")
    i += 1

        
