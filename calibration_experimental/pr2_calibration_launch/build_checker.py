#!/usr/bin/env python

import sys
import os
import string

def print_usage():
    print '''Usage:
    build_checker.py <num_x> <num_y> <square_size>
'''

if len(sys.argv) < 4:
    print_usage();

num_x   = int(sys.argv[1])
num_y   = int(sys.argv[2])
sq_size = float(sys.argv[3])

print '''<?xml version="1.0" ?>
<!-- Object2: example checker box -->
<model:physical name="checker_model">
  <xyz>   0.0    0.0    2.0 </xyz>
  <rpy>   0.0    0.0    0.0</rpy>
  <static>true</static>
  <body:box name="checker_body">'''

for y in range(0, num_y):
    for x in range(0, num_x):
        if y%2 == x%2:
            laser_retro = 3000
            material_str = "PR2/White"
        else:
            laser_retro = 1000
            material_str = "PR2/Black"
        box_name_str = "checker_%u%u_geom" % (x,y)
        position_str = "0.0 %.3f %.3f" % (x*sq_size, y*sq_size)
        size_str     = "%.3f %.3f %.3f" % (sq_size, sq_size, sq_size)
        print '''
    <geom:box name="%s">
      <xyz>%s</xyz>
      <size>%s</size>
      <laserRetro>%u</laserRetro>
      <kp>1000000.0</kp>
      <kd>1.0</kd>
      <mesh>default</mesh>
      <mass> 0.01</mass>
      <visual>
        <size>%s</size>
        <material>%s</material>
        <mesh>unit_box</mesh>
      </visual>
    </geom:box>''' % (box_name_str, position_str, size_str, laser_retro, size_str, material_str)

print '''  </body:box>
  <controller:ros_p3d name="p3d_object_controller" plugin="libros_p3d.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>100.0</updateRate>
    <bodyName>checker_body</bodyName>
    <topicName>checker_pose_ground_truty</topicName>
    <frameName>map</frameName>
    <interface:position name="p3d_object_position"/>
  </controller:ros_p3d>
</model:physical>'''
