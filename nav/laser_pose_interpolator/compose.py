#!/usr/bin/env python

import sys

d = False
out = open("compose.txt", "w")
x = 0.0
y = 0.0
time = 0.0
xpos = 0.0
ypos = 0.0
starttime = 0.0
for a in sys.argv:
    if (d):
        print a
        print xpos
        print ypos
        f = open(a, "r")
        for line in f:
            sp = line.split(" ")
            if (sp[0] == "FLASER"):
                n = int(sp[1])
                sp[n + 2] = float(sp[n + 2]) + xpos
                sp[n + 3] = float(sp[n + 3]) + ypos
                sp[n + 5] = float(sp[n + 5]) + xpos
                sp[n + 6] = float(sp[n + 6]) + ypos
                sp[n + 8] = float(sp[n + 8]) + starttime
                sp[n + 10] = float(sp[n + 10]) + starttime
                x = float(sp[n + 2])
                y = float(sp[n + 3])
                time = float(sp[n + 10])
                #print str(x) + " " + str(y)
            elif (sp[0] == "ODOM"):
                sp[1] = float(sp[1]) + xpos
                sp[2] = float(sp[2]) + ypos
                sp[7] = float(sp[7]) + starttime
                sp[9] = float(sp[9]) + starttime
                x = float(sp[1])
                y = float(sp[2])
                time = float(sp[7])
            space = ""
            for s in sp:
                if (space != ""):
                    out.write(space)
                   #print s
                out.write(str(s))
                space = " "
            out.write("\n")
        f.close()
        xpos = x
        ypos = y
        starttime = time
        print xpos
        print ypos
    else:
        d = True

out.close()
