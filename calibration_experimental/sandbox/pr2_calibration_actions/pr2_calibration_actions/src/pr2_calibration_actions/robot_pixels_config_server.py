#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_calibration_actions')
from pr2_calibration_actions.srv import *
from pr2_calibration_actions.msg import *
import rospy
import yaml

global data_map
global config_msg

def handle_get_robot_pixels_config(req):
    global data_map
    rospy.loginfo("In Service Call")
    resp = GetRobotPixelsConfigResponse()
    resp.config = config_msg
    resp.command = [float(x) for x in data_map['commands'][req.command_num]]
    return resp

def loadJointStatesChannelConfig(local_map):
    #local_map['tolerances'] = [float(x) for x in local_map['tolerances']]
    local_map['padding'] = rospy.Duration(local_map['padding'])
    #local_map['min_samples'] = int(local_map['min_samples'])
    #local_map['cache_size'] = int(local_map['cache_size'])
    return JointStatesChannelConfig(**local_map)

def loadPixelChannelConfig(local_map):
    local_map['padding'] = rospy.Duration(local_map['padding'])
    return PixelChannelConfig(**local_map)

def loadCrossPixelTimeshift(local_map):
    local_map['max_timeshift'] = rospy.Duration(local_map['max_timeshift'])
    return CrossPixelTimeshift(**local_map)

def load_capture_robot_pixels_goal(local_map):
    goal = CaptureRobotPixelsGoal()

    goal.joint_states_config = loadJointStatesChannelConfig(local_map['joint_states_config'])

    goal.pixel_configs = [loadPixelChannelConfig(x) for x in local_map['pixel_configs'] ]
    goal.joint_states_timeshift = rospy.Duration(local_map['joint_states_timeshift'])
    goal.pixel_timeshifts = [loadCrossPixelTimeshift(x) for x in local_map['pixel_timeshifts']]
    return goal

def robot_pixels_config_server():
    global data_map
    global config_msg
    rospy.init_node('robot_pixels_config_server')

    if rospy.has_param('~config'):
        rospy.loginfo("Found config")
    else:
        rospy.logfatal("Could not find parameter [~config] on param server")

    config = rospy.get_param('~config', ' ')

    data_map = yaml.load(config)

    config_msg = load_capture_robot_pixels_goal(data_map['config'])

    s = rospy.Service('get_robot_pixels_config', GetRobotPixelsConfig, handle_get_robot_pixels_config)
    rospy.spin()

if __name__ == "__main__":
    robot_pixels_config_server()

