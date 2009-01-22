#!/usr/bin/env python

PKG = 'vision_utils'
import rostools; rostools.update_path(PKG)
import rospy
import rosrecord


def sortbags(inbag, outbag):
  rebag = rosrecord.Rebagger(outbag)

  schedule = []
  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    if rospy.is_shutdown():
      break
    schedule.append((t, i))
  schedule = [ i for (t,i) in sorted(schedule) ]
  print schedule

  stage = {}
  for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
    if rospy.is_shutdown():
      break
    stage[i] = (topic, msg, t)
    while (schedule != []) and (schedule[0] in stage):
      (topic, msg, t) = stage[schedule[0]]
      rebag.add(topic, msg, t, raw=True)
      del stage[schedule[0]]
      schedule = schedule[1:]
  assert schedule == []
  assert stage == {}
  rebag.close()

if __name__ == '__main__':
  import sys
  if len(sys.argv) == 3:
    sortbags(sys.argv[1], sys.argv[2])
  else:
    print "usage: bagsort <inbag> <outbag>"
