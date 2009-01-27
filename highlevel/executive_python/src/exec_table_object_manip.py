#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Brian Gerkey (modelled after Eitan's executive.py)

# States:
#
# IDLE - this is the start state
#
# SLOWSCAN - not moving, slow-scanning
#
# MOVEBASE - moving base to goal, slow-scanning
# 
# MOVETOGRASP - moving EE to goal near object, fast-scanning
#
# GRASPING - closing gripper
#
# RAISING - moving EE to goal above table, no EE collision detection
#
# STOWING - moving EE to pre-defined stow configuration, fast-scanning, 
#           object joined to robot, subtracted from map
#
# MOVETODROP - moving EE to goal near landing zone, fast-scanning, object
#              joined to robot, subtracted from map (same as STOWING, with
#              different goal?)
#
# DROPPING - opening gripper
#

# Fault-free state sequence:
#
# 1. IDLE
#
# 2. SLOWSCAN (look for table / objects)
#  Collision map returns table geometry and list of object geometries.
#  Pick an object
#  Compute a goal base pose near the table & object
#
# 3. MOVEBASE (go to table)
#  Send goal base pose to MoveBase
#
# 4. SLOWSCAN (look for table / objects)
#  Collision map returns table geometry and list of object geometries.
#  Pick an object (pick the same one somehow?)
#
# 5. MOVETOGRASP
#  Send object pose as constraint-based EE goal to MoveArm
#
# 6. GRASPING
#  Blindly close, grasping object
#
# 7. RAISING
#  Disable EE collision body
#  Compute EE goal fixed distance in +Z from table surface (remember table
#    geometry from step 4; could SLOWSCAN again if necessary)
#  Give EE goal to MoveArm
#  Enable EE collision body
#  Remove object geom from collision map
#  Add object geom to robot collision model (AttachedObject message)
#
# 8. STOWING
#  Send pre-defined stow configuration to MoveArm
#
# 9. SLOWSCAN (look for table / landing zone)
#  Collision map returns table geometry and convex hulls for empty
#    regions
#  Pick empty region
#  Compute point in region
#  Compute goal base pose near point & table
#
#10. MOVEBASE (go to landing zone)
#  Send goal base pose to MoveBase
#
#11. SLOWSCAN
#  Validate chosen landing zone
#  Compute EE goal pose 
#  
#12. MOVETODROP
#  Send EE goal pose to MoveArm
#
#13. DROPPING
#  Blindly open gripper, releasing object
#  Remove object from body (empty AttachedObject list)
#
#14. STOWING
#  Send pre-defined stow configuration to MoveArm
#
#15. Goto (1)

import rostools
rostools.load_manifest('executive_python')
import rospy
import random
from robot_msgs.msg import VisualizationMarker
from robot_msgs.msg import AttachedObject, PoseConstraint
from robot_srvs.srv import FindTable, FindTableRequest, SubtractObjectFromCollisionMap, SubtractObjectFromCollisionMapRequest, RecordStaticMapTrigger, RecordStaticMapTriggerRequest
from pr2_mechanism_controllers.srv import SetProfile, SetProfileRequest
from highlevel_controllers.msg import *
from navigation_adapter import *
from movearm_adapter import *
#from tiltlaser_adapter import *
#from gripper_adapter import *

class Executive:
  def __init__(self, navigator, armigator, cycle_time):
    rospy.init_node('Executive', anonymous=True)
    self.navigator = navigator
    self.armigator = armigator
    self.cycle_time = cycle_time
    self.state = 'idle'
    self.current_goal = None
    self.current_object = None

    # Hackety hack hack
    self.vmk_id = 15000
    self.object_padding = 0.03
    self.laser_buffer_time = 5.0
    # Should be smaller, but that throws out some graspable objects
    self.gripper_max_object_diam = 0.09
    self.robot_name = 'pr2'
    self.gripper_link_name = 'r_gripper_palm_link'

    self.first_time = True
    self.scan_start_time = -1.0

    self.laser_tilt_profile_type = 4 # Implies sine sweep
    self.laser_tilt_profile_controller = "laser_tilt_controller"
    self.laser_tilt_profile_period_fast = 2.0
    self.laser_tilt_profile_period_slow = 10.0
    self.laser_tilt_profile_amplitude = 0.75
    self.laser_tilt_profile_offset = 0.30

    self.vis_pub = rospy.Publisher("visualizationMarker", VisualizationMarker)
    self.attached_obj_pub = rospy.Publisher("attach_object", AttachedObject)

    print 'Waiting for service: ' + self.laser_tilt_profile_controller + '/set_profile'
    rospy.wait_for_service(self.laser_tilt_profile_controller + '/set_profile')
    print 'Waiting for service: collision_map_buffer/subtract_object'
    rospy.wait_for_service('collision_map_buffer/subtract_object')
    print 'Waiting for service: table_object_detector'
    rospy.wait_for_service('table_object_detector')
    print 'Waiting for service: collision_map_buffer/record_static_map'
    rospy.wait_for_service('collision_map_buffer/record_static_map')

  def legalStates(self):
    return True
    if not self.navigator.legalState():
      print("Waiting on %s to be published" % (self.navigator.state_topic))
      rospy.logout("Waiting on %s to be published" % (self.navigator.state_topic))
    if not self.armigator.legalState():
      print("Waiting on %s to be published" % (self.navigator.state_topic))
      rospy.logout("Waiting on %s to be published" % (self.navigator.state_topic))
    return self.navigator.legalState() and self.armigator.legalState()

  def adaptTiltSpeed(self, period):

    #print 'Sending Command: '
    #print '  Profile Type: %d' % self.laser_tilt_profile_type
    #print '  Period:       %f Seconds' % period
    #print '  Amplitude:    %f Radians' % self.laser_tilt_profile_amplitude
    #print '  Offset:       %f Radians' % self.laser_tilt_profile_offset

    s = rospy.ServiceProxy(self.laser_tilt_profile_controller + '/set_profile', SetProfile)
    resp = s.call(SetProfileRequest(0.0, 0.0, 0.0, 0.0, 
                                    self.laser_tilt_profile_type, 
                                    period, 
                                    self.laser_tilt_profile_amplitude, 
                                    self.laser_tilt_profile_offset))
        
    #print 'Command Sent'
    #print '  Response: %s' % str(resp.time)

    # Set the collision_map_buffer's window size accordingly, to remember a
    # fixed time window scans.
    rospy.client.set_param('collision_map_buffer/window_size', 
                           int(self.laser_buffer_time / (period / 2.0)))
    return resp

  def getTable(self):
    print 'Calling FindTable'
    s = rospy.ServiceProxy('table_object_detector', FindTable)
    resp = s.call(FindTableRequest())
    print 'Table: %f %f %f %f' % (resp.table.min_x,
                                  resp.table.max_x,
                                  resp.table.min_y,
                                  resp.table.max_y)
    print '%d objects' % len(resp.table.objects)
    for o in resp.table.objects:
      print '  (%f %f %f): %f %f %f' % \
         (o.center.x, o.center.y, o.center.z,
          o.max_bound.x - o.min_bound.x,
          o.max_bound.y - o.min_bound.y,
          o.max_bound.z - o.min_bound.z)

    return resp

  def recordStaticMap(self, t):
    print 'Calling record_static_map'
    s = rospy.ServiceProxy('collision_map_buffer/record_static_map', RecordStaticMapTrigger)
    resp = s.call(RecordStaticMapTriggerRequest(t))
    print 'response: %d' % resp.status

  def drawObjectVisMarker(self, obj):
    # TODO: use bounds instead of this hardcoded radius
    radius = 2.0

    mk = VisualizationMarker()
    mk.header.frame_id = obj.frame_id
    mk.id = self.vmk_id
    #self.vmk_id += 1
    mk.type = VisualizationMarker.SPHERE
    mk.action = VisualizationMarker.ADD # same as modify
    mk.x = obj.center.x
    mk.y = obj.center.y
    mk.z = obj.center.z
    mk.roll = mk.pitch = mk.yaw = 0
    mk.xScale = mk.yScale = mk.zScale = radius * 2.0
    mk.alpha = 255
    mk.r = 255
    mk.g = 10
    mk.b = 10

    vis_pub.publish(mk)

  def findLargestObject(self, objs):
    largest_obj = None
    largest_size = -1.0
    i = 0
    for obj in objs:
      dx = (obj.max_bound.x - obj.min_bound.x)
      dy = (obj.max_bound.y - obj.min_bound.y)
      dz = (obj.max_bound.z - obj.min_bound.z)
      if dx > self.gripper_max_object_diam or \
         dy > self.gripper_max_object_diam:
       print 'object too large'
       continue
      sz = dx*dx + dy*dy + dz*dz;
      if sz > largest_size:
        largest_obj = obj
        largest_size = sz
    return largest_obj

  def padObject(self, obj):
    print 'Old bounds: (%f %f %f) <-> (%f %f %f)' % \
      (obj.min_bound.x, obj.min_bound.y, obj.min_bound.z,
       obj.max_bound.x, obj.max_bound.y, obj.max_bound.z)

    #dx = (obj.max_bound.x - obj.min_bound.x)
    #dy = (obj.max_bound.y - obj.min_bound.y)
    #dz = (obj.max_bound.z - obj.min_bound.z)
    #maxd = dx
    #if dy > maxd:
    #  maxd = dy
    #if dz > maxd:
    #  maxd = dz

    # Account for parts that aren't seen (e.g., bottom of object, which
    # gets cut off)
    #maxd += self.object_padding
    obj.min_bound.z -= self.object_padding

    #obj.min_bound.x = obj.center.x - 1.5 * maxd / 2.0
    #obj.min_bound.y = obj.center.y - 1.5 * maxd / 2.0
    #obj.min_bound.z = obj.center.z - 1.5 * maxd / 2.0
    #obj.max_bound.x = obj.center.x + 1.5 * maxd / 2.0
    #obj.max_bound.y = obj.center.y + 1.5 * maxd / 2.0
    #obj.max_bound.z = obj.center.z + 1.5 * maxd / 2.0

    print 'New bounds: (%f %f %f) <-> (%f %f %f)' % \
      (obj.min_bound.x, obj.min_bound.y, obj.min_bound.z,
       obj.max_bound.x, obj.max_bound.y, obj.max_bound.z)

    return obj

  def subtractObjectFromCollisionMap(self, header, obj):
    s = rospy.ServiceProxy('collision_map_buffer/subtract_object',
SubtractObjectFromCollisionMap)
    resp = s.call(SubtractObjectFromCollisionMapRequest(header, obj))
    return resp

  def attachObjectToRobot(self, header, obj):
    m = AttachedObject()
    m.header = header
    m.robot_name = self.robot_name
    m.link_name = self.gripper_link_name
    m.objects = [obj]
    self.attached_obj_pub.publish(m)

  def handleIdle(self, t):  
    #if self.navigator.goalReached() or (not self.navigator.active() and self.current_goal == None) or self.navigator.timeUp():
    #  self.current_goal = self.goals[random.randint(0, len(self.goals) - 1)]
    #  self.navigator.sendGoal(self.current_goal, "odom")
    #  print "nav --> nav"
    #elif not self.navigator.active() and self.current_goal != None:
    #  self.navigator.sendGoal(self.current_goal, "odom")
    #  print "nav --> nav"
    # TRANSITION: idle -> slowscan
    return 'slowscan'

  def handleSlowScan(self, t):  
    # Did we start the scan yet?
    if(self.scan_start_time < 0.0):
      # Request slow scan & trigger static map recording
      resp = self.adaptTiltSpeed(self.laser_tilt_profile_period_slow)
      self.scan_start_time = t
      if self.first_time:
        self.recordStaticMap(rostools.rostime.Time().from_seconds(resp.time))
        self.first_time = False

    #print 'Waiting for slow scan to complete...'
    # Hack
    if (t - self.scan_start_time) >= .75*self.laser_tilt_profile_period_slow:
      #print '...done'
      resp = self.getTable()
      obj = self.findLargestObject(resp.table.objects)
      #self.drawObjectVisMarker(obj)
      if obj == None:
        print 'Error: no object found!'
      else:
        print 'Chose object at (%f %f %f)' % (obj.center.x,
                                              obj.center.y,
                                              obj.center.z)
        self.current_object = self.padObject(obj)

        # Subtract object from cmap
        self.subtractObjectFromCollisionMap(resp.table.header, self.current_object)

        # Attach the object to the robot body
        self.attachObjectToRobot(resp.table.header, self.current_object)

        self.scan_start_time = -1.0
        # TRANSITION: slowscan -> fastscan
        return 'fastscan'
    else:
      return self.state

  def handleFastScan(self, t):  
    # Did we start the scan yet?
    if(self.scan_start_time < 0.0):
      # Request fast scan
      self.adaptTiltSpeed(self.laser_tilt_profile_period_fast)
      self.scan_start_time = t

    #print 'Waiting for fast scan to complete...'
    # Hack
    if (t - self.scan_start_time) >= 2.0*self.laser_tilt_profile_period_fast:
      #print '...done'
      # TRANSITION: fastscan -> idle
      self.scan_start_time = -1.0
      return 'idle'
    else:
      return self.state

  def handleMoveToGrasp(self, t):
    if self.current_object == None:
      print 'No object chosen to grasp!'
    else:
      c1 = PoseConstraint()
      c1.type = PoseConstraint.COMPLETE_POSE
      c1.robot_link = 'r_gripper_palm_link'
      c1.pose.position.x = 0.0
      c1.pose.position.y = 0.0
      c1.pose.position.z = 0.0
      c1.pose.orientation.x = 0.0
      c1.pose.orientation.y = 0.0
      c1.pose.orientation.z = math.sqrt(2.0)/2.0
      c1.pose.orientation.w = math.sqrt(2.0)/2.0

      c2 = PoseConstraint()
      c3 = PoseConstraint()
      constraints = [c1, c2, c3]
      armigator.sendGoal(current_object.header.frame_id,
                         True,
                         None,
                         constraints,
                         1,
                         30.0)

  def handleDone(self, t):
    print 'All done.'
    sys.exit(0)

  def doCycle(self):
    curr_time = rospy.rostime.get_time()
    #make sure that all adapters have legal states
    if self.legalStates():
      if self.state == 'idle':
        self.state = self.handleIdle(curr_time)

      elif self.state == 'slowscan':
        self.state = self.handleSlowScan(curr_time)

      elif self.state == 'fastscan':
        self.state = self.handleFastScan(curr_time)

      elif self.state == 'done':
        self.state = self.handleDone(t)
      
      else:
        print 'Invalid state: ' % self.state
        sys.exit(-1)


  def run(self):
    while not rospy.is_shutdown():
      start = rospy.get_time()
      self.doCycle()
      end = rospy.get_time()
      sleep_time = self.cycle_time - (end - start)
      if sleep_time > 0:
        rospy.sleep(sleep_time)
      else:
        print("Executive missed cycle time of %.2f seconds by %.3f seconds" % (self.cycle_time, -sleep_time))
        rospy.logwarn("Executive missed cycle time of %.2f seconds by %.3f seconds" % (self.cycle_time, -sleep_time))

if __name__ == '__main__':
  try:
    navigator = NavigationAdapter(30, 300, 'state', 'goal')
    armigator = MoveArmAdapter(30, 30, 'right_arm_state', 'right_arm_goal')

    executive = Executive(navigator, armigator, 1.0)
    executive.run()
  except KeyboardInterrupt, e:
    pass
  print "exiting"


