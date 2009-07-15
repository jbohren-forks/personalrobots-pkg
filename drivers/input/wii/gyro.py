#! /usr/bin/python
import cwiid
import operator
import time
import sys
from math import *

class Gyro:
    def __init__(self):
         print >> sys.stderr, "Press both buttons to pair."
         self.wm = cwiid.Wiimote()
         print >> sys.stderr, "Pairing successful."
         self.wm.enable(cwiid.FLAG_MOTIONPLUS)
         self.wm.rpt_mode = cwiid.RPT_ACC | cwiid.RPT_MOTIONPLUS | cwiid.RPT_NUNCHUK
         time.sleep(0.2)
         print >> sys.stderr, "Wimotion activated."

    def setcallback(self, f):
         self.wm.mesg_callback = f
         self.wm.enable(cwiid.FLAG_MESG_IFC)

    def clearcallback(self):
         self.wm.disable(cwiid.FLAG_MESG_IFC)

    def zerocallback(self, state, time):
        self.zero_count = self.zero_count + 1;
        newdat = state[1][1]['angle_rate']
        for j in range(0, 3):
            self.sum[j] = self.sum[j] + newdat[j]
            self.sumsqr[j] = self.sumsqr[j] + newdat[j] * newdat[j]
        if self.zero_count >= self.zero_N:
            self.clearcallback()
            self.mean = []
            self.var = []
            self.dev = []
            for j in range(0, 3):
                 self.mean.append(float(self.sum[j]) / self.zero_N)
                 self.var.append(float(self.sumsqr[j]) / self.zero_N - self.mean[j] * self.mean[j])
                 try:
                     self.dev.append(sqrt(self.var[j]))
                 except:
                     self.dev.append(0)
                     print >> sys.stderr, "sqrt error: ", self.var[j]

            print >> sys.stderr, "Mean:  ", self.mean
            print >> sys.stderr, "Stdev: ", self.dev
            self.zero_done = True

    def zero(self):
         print >> sys.stderr, "Zeroing..."
         self.sum = [0,0,0]
         self.sumsqr = [0,0,0]
         self.zero_N = 100
         self.setcallback(self.zerocallback)
         self.zero_count = 0;
         self.zero_done = False;
         while self.zero_done == False:
             time.sleep(.2)

    def integratecallback(self, state, time):
        newdat = state[1][1]['angle_rate']
        self.count = self.count + 1
        delta = [0., 0., 0.]
        for j in range(0, 3):
            delta[j] = newdat[j] - self.mean[j]
            self.pos[j] = self.pos[j] + delta[j]
        print delta
        if self.count >= 200:
#            print "Pos: ", self.pos
            self.count = 0
            self.pos = [0., 0., 0.]

    def integrate(self):
        self.pos = [0., 0., 0.]
        self.setcallback(self.integratecallback)
        self.count = 0
    
    def dumpcallback(self, state, time):
        newdat = state[1][1]['angle_rate']
        for i in range(0,3):
            print newdat[i],
        print

    def dump(self):
        self.setcallback(self.dumpcallback)


g = Gyro()
#g.dump()
while True:
  g.zero()
  for i in range(0, 3):
      print g.mean[i],
  print
  sys.stdout.flush()
g.integrate()

while True:
    time.sleep(.2)

