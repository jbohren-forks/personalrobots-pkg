#! /usr/bin/env python
# Provides quick access to the services exposed by MechanismControlNode

import roslib, time
roslib.load_manifest('mechanism_control')

import rospy, sys
from mechanism_control import mechanism
from mechanism_msgs.msg import MechanismState


class Tracker:
  def __init__(self, topic, Msg):
    self.sub = rospy.Subscriber(topic, Msg, self.callback)
    self.msg = None #Msg()

  def callback(self, msg):
    self.msg = msg

def print_usage(exit_code = 0):
    print '''Commands:
    list          - List active controllers
    spawn         - Spawn and start a controller using the xml passed over stdin
    spawn-stopped - Spawn a controller without starting it using the xml passed over stdin
    kill <name>   - Kills the controller named <name>
    start <name>  - Starts the controller named <name>
    stop <name>   - Stops the controller named <name>
    shutdown      - Ends whole process
    list-joints   - List joints and actuators
    list-types    - List controller Types'''

    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()
    if sys.argv[1] == 'lt' or sys.argv[1] == 'list-types':
        mechanism.list_controller_types()
    elif sys.argv[1] == 'lc' or sys.argv[1] == 'list':
        mechanism.list_controllers()
    elif sys.argv[1] == 'start':
        for c in sys.argv[2:]:
          mechanism.start_stop_controller(c, True)
    elif sys.argv[1] == 'stop':
        for c in sys.argv[2:]:
          mechanism.start_stop_controller(c, False)
    elif sys.argv[1] == 'sp' or sys.argv[1] == 'spawn':
        xml = ""
        if len(sys.argv) > 2:
            f = open(sys.argv[2])
            xml = f.read()
            f.close()
        else:
            xml = sys.stdin.read()
        mechanism.spawn_controller(xml, 1)
    elif sys.argv[1] == 'spawn-stopped':
        xml = ""
        if len(sys.argv) > 2:
            f = open(sys.argv[2])
            xml = f.read()
            f.close()
        else:
            xml = sys.stdin.read()
        mechanism.spawn_controller(xml, 0)
    elif sys.argv[1] == 'kl' or sys.argv[1] == 'kill':
        for c in sys.argv[2:]:
            mechanism.kill_controller(c)
    elif sys.argv[1] == 'shutdown':
        mechanism.shutdown()
    elif sys.argv[1] == 'lj' or sys.argv[1] == 'list-joints':
        track_mech = Tracker('/mechanism_state', MechanismState)
        print "Waiting for mechanism state message..."
        rospy.init_node('mech', anonymous=True)
        while not track_mech.msg:
            time.sleep(0.01)
            if rospy.is_shutdown():
                sys.exit(0)
        msg = track_mech.msg
        print 'Actuators:'
        for i in range(len(msg.actuator_states)):
            print "  %3d %s" % (i, msg.actuator_states[i].name)
        print 'Joints:'
        for i in range(len(msg.joint_states)):
            print "  %3d %s" % (i, msg.joint_states[i].name)
    else:
        print_usage(1)
