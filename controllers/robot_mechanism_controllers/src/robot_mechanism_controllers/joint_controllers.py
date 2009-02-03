from controller import Controller
import roslib
import rospy

from srv import *

## JointVelocityController

class JointVelocityProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self, topic_name)
    
    self.set_ = rospy.ServiceProxy('%s/set_command' % self.topic_name, SetCommand).call 
    self.get_ = rospy.ServiceProxy('%s/get_actual' % self.topic_name, GetActual).call
    
  def get(self, v):
    return self.get_(GetActualRequest(v))

  def set(self, v):
    return self.set_(SetCommandRequest(v))

## JointPositionController

class JointPositionProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self,topic_name)
    self.set_ = rospy.ServiceProxy('/%s/set_command' % self.topic_name, SetCommand).call
    self.get_ = rospy.ServiceProxy('/%s/get_actual' % self.topic_name, GetActual).call

  def get(self, v):
    return self.get_(GetActualRequest(v))

  def set(self, v):
    return self.set_(SetCommandRequest(v))

class JointManualCalibrationProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self, topic_name)
    #self.set_offset = rospy.ServiceProxy('/%s/set_offset' % self.topic_name, SetJointOffset).call
    #self.get_offset = rospy.ServiceProxy('/%s/get_offset' % self.topic_name, GetJointOffset).call
    self.begin_manual_ = rospy.ServiceProxy('/%s/begin_manual_calibration' % self.topic_name, CalibrateJoint).call
    self.end_manual_ = rospy.ServiceProxy('/%s/end_manual_calibration' % self.topic_name, CalibrateJoint).call
    
  def begin_manual(self):
    return self.begin_manual_(CalibrateJointRequest())

  def end_manual(self):
    return self.end_manual_(CalibrateJointRequest())

class JointCalibrationProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self, topic_name)
    #self.set_offset = rospy.ServiceProxy('/%s/set_offset' % self.topic_name, SetJointOffset).call
    #self.get_offset = rospy.ServiceProxy('/%s/get_offset' % self.topic_name, GetJointOffset).call
    self.calibrate_ = rospy.ServiceProxy( '/%s/calibrate' % self.topic_name, CalibrateJoint).call

  def calibrate(self):
    return self.calibrate_(CalibrateJointRequest())

class JointBlindCalibrationProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self, topic_name)
    #self.set_offset = rospy.ServiceProxy('/%s/set_offset' % self.topic_name, SetJointOffset).call
    #self.get_offset = rospy.ServiceProxy('/%s/get_offset' % self.topic_name, GetJointOffset).call
    self.calibrate_ = rospy.ServiceProxy( '/%s/calibrate' % self.topic_name, CalibrateJoint).call

  def calibrate(self):
    return self.calibrate_(CalibrateJointRequest())


class JointAutotunerProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self, topic_name)
    
   
JointAutotuner = JointAutotunerProxy

## JointPositionController

class JointSineSweepProxy(Controller):
  def __init__(self, topic_name):
    Controller.__init__(self,topic_name)
    print '/%s/begin_sweep' % self.topic_name
    self.sweep_ = rospy.ServiceProxy('/%s/begin_sweep' % self.topic_name, SetCommand).call

  def sweep(self, start_freq, end_freq, duration, amplitude):
    return self.sweep_(BeginJointSineSweepRequest(start_freq, end_freq, duration, amplitude))
