#
# converts toro files to incremental form
# first arg is input toro file
# second arg is output toro file
#

#
# file lines have this form:
# VERTEX Vn xxxxx, ordered by Vn
# EDGE3 V1 V2 xxxxx, with V1 < V2
#
# puts all edges just after the highest-numbered vertex
#


import numpy
import math
import random
import time
import sys


if len(sys.argv) < 3:
    print "Arguments:  <input Toro file>\n"
    print "            <output ordered Toro file>\n"
    sys.exit(0)
    

print sys.argv[1]
print sys.argv[2]


# open files
ifile = open(sys.argv[1],'r');
ofile = open(sys.argv[2],'w');


# read in all lines and store in vertex list and edge dict
vs = []                                 # list of vertices
es = {}                                 # dict of edges
for line in ifile:
    a = line.split()
    if (a[0] == "VERTEX3"):             # vertex line
#        print "VERTEX3 "+a[1]
        vs.append(line);

    if (a[0] == "EDGE3"):
#        print "EDGE3 "+a[1]+" "+a[2]
        if a[2] in es:
            es[a[2]].append(line)
        else:
            es[a[2]] = [line]


for v in vs:
    ofile.write(v)
    a = v.split()
    if a[1] in es:
        for e in es[a[1]]:
            ofile.write(e)


print "File written"



        
