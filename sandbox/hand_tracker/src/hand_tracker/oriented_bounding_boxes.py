#!/usr/bin/python

import roslib; roslib.load_manifest('hand_tracker')
import Image
import xml.dom.minidom
import os
import cv
from numpy import *
import random

#----------------------------------------------------------------------
#                         Helper functions
#----------------------------------------------------------------------
def dot_product(A, B):
  return sum(A*B)

# L2 norm of a vector
def norm(A):
  return sqrt(sum(A*A))

def getCorners(oriented_box):
  x,y,W,H,alpha,flip = oriented_box
  c = cos(alpha)
  s = sin(alpha)
  return [(x+c*W/2-s*H/2, y+s*W/2+c*H/2),
          (x-c*W/2-s*H/2, y-s*W/2+c*H/2),
          (x-c*W/2+s*H/2, y-s*W/2-c*H/2),
          (x+c*W/2+s*H/2, y+s*W/2-c*H/2)]

#----------------------------------------------------------------------
#                          Core functions
#----------------------------------------------------------------------

# Assumes a 7 point list as input, where each element of the list is a
# numpy array, representing the position of the tips of the fingers
# starting from the thumb, and the base of the hand on each side of the
# wrist.
# Returns an oriented bounding box defined by a 5-tuple:
# (x, y, width, height, angle, flip)
# where (x, y) is the middle of the box, expressed in image coordinates.
# flip is 0 if the thumb is on the left, 1 if it is on the right
#
# margin is a proportion increase of the size of the bounding box (to
# capture some context and be sure not to cut part of the object)
# for instance 0.1 increases the box size by 10%, ie 5% on each side
#
# aspect ration is the deisired height/width. The box will be expanded
# to reach this ratio.
def single_hand_outline_to_oriented_box(points, margin, aspect_ratio):
  # Check input
  if (len(points) != 7):
    return None
  
  # Compute axis from middle of the base of the hand to the tip of the middle finger
  M = points[2]  # Tip of middle finger
  A = points[5]  # Base of hand, pinky side
  B = points[6]  # Base of hand, thumb side
  C = (A+B)/2    # Base of hand, middle
  v = M-C  # vector arithmetic (using numpy)
  v = v/norm(v)  # v is now a unit vector
  u = array([v[1], -v[0]], dtype=float)  # u,v: local frame for the hand, in image coordinates

  #print C
  #print M
  #print u
  #print v

  # Convert to local coordinates in frame defined by (C,u,v)
  points_local = []
  for P in points:
    p = P - C
    x = dot_product(p, u)  # Position of P perpendicular to the (M,C) axis
    y = dot_product(p, v)  # Position of P along the (M,C) axis
    points_local.append(array([x,y], dtype=float))

  # Determine which side the thumb is
  if (points_local[0][0] > 0):
    flip = 1  # Is the x coordinate of the thumb in local frame (which is centered at the base of the hand) positive?
  else:
    flip = 0

  # Compute bounding box in local coordinates
  lower_left = array([inf, inf])
  upper_right = array([-inf, -inf])
  for P in points_local:
    lower_left = minimum(lower_left,P)  # element-wise operation
    upper_right = maximum(upper_right, P)      
  box_center = (upper_right + lower_left)/2
  width_height = upper_right - lower_left
  width_height *= (1.0+margin)
  if width_height[1]/width_height[0] > aspect_ratio:
    width_height[0] = width_height[1] / aspect_ratio
  else:
    width_height[1] = width_height[0] * aspect_ratio

  # The image frame, expressed in local coordinates ('r' is for 'reverse')
  ru = array([u[0], -u[1]])
  rv = array([-ru[1], ru[0]])

  #print ru
  #print rv

  # Find the box center in image coordinates
  # (though the names 'top' and 'left' refer to the position in the local frame).
  x = dot_product(box_center, ru)
  y = dot_product(box_center, rv)
  box_center_in_image = C + array([x,y])  # top_left is in image coordinates, while upper_left is in local coordinates

  # Return the box
  # TODO: EXPLAIN BETTER
  # We give pi minus the angle because the coordinates of the image have (0,0) in the
  # top left corner, thus we are doing trigonometric operations on the upside-
  # down image. A vertical hand will appear upside down, so we need to take minus
  # the angle if we want the angle with respect to the correct orientation, and add pi
  # because the vertical has also been flipped.
  return box_center_in_image.tolist() + width_height.tolist() + [pi-arctan2(u[1],u[0]), flip]


# The results from the Mechanical Turk task to label hands returns results in a nested way,
# with a loose bounding box for each hand, then a polygon whose coordinates are given
# relative to the bounding box. This function converts it to (absolute) image coordinates.
# THE CONVERSION PERFORMED BY THIS FUNCTION IS NO LONGER NEEDED, IT HAS BEEN DISABLED
# SO THIS FUNCTION SIMPLY EXTRACTS <oriented_box> ANNOTATIONS
def convert_polygons_to_absolute_coord(xml_filename):
  # Open empty xml document to receive the converted data
  output_doc = xml.dom.minidom.getDOMImplementation().createDocument(None, "annotations", None)
  node_1 = output_doc.createElement("results")
  node_2 = output_doc.createElement("annotation")
  output_doc.documentElement.appendChild(node_1)
  node_1.appendChild(node_2)

  # Read the input xml and convert
  input_doc = xml.dom.minidom.parse(xml_filename)
  for bbox in input_doc.getElementsByTagName("bbox"):
    o = array([float(bbox.getAttribute("left")), float(bbox.getAttribute("top"))], dtype=float)
    # The size of the bounding box, in image coordinates
    w = float(bbox.getAttribute("width"))
    h = float(bbox.getAttribute("height"))
    #print w
    #print h
    # The resolution at which the bounding box was displayed (usually 640x480)
    res = bbox.getElementsByTagName("size")[0]
    W = float(res.getElementsByTagName("width")[0].childNodes[0].data)
    H = float(res.getElementsByTagName("height")[0].childNodes[0].data)
    #print W
    #print H
    # The scaling ratio between the two
    s = max(w/W, h/H)
    #print s
    #print o
    for polygon in bbox.getElementsByTagName("polygon"):
      output_polygon = output_doc.createElement("polygon")
      node_2.appendChild(output_polygon)
      for pt in polygon.getElementsByTagName("pt"):
        output_pt = output_doc.createElement("pt")
        output_polygon.appendChild(output_pt)
        x = float(pt.getAttribute("x"))
        y = float(pt.getAttribute("y"))
        P = array([x, y], dtype=float)
        #P = P*s + o  # DO NOT PROCESS
        output_pt.setAttribute("x", str(P[0]))
        output_pt.setAttribute("y", str(P[1]))
  return output_doc.toprettyxml()


# Find all the polygons in an xml file, and transform them to
# containing oriented bounding boxes expressed in xml.
def convert_polygons_to_oriented_boxes(xml_filename, margin, aspect_ratio):
  # Open xml file and read results:
  input_doc = xml.dom.minidom.parse(xml_filename)
  oriented_boxes = []
  for polygon in input_doc.getElementsByTagName("polygon"):
    points = []
    for pt in polygon.getElementsByTagName("pt"):
      x = float(pt.getAttribute("x"))
      y = float(pt.getAttribute("y"))
      P = array([x, y], dtype=float)
      points.append(P)
    oriented_boxes.append(single_hand_outline_to_oriented_box(points, margin, aspect_ratio))

  # Describe the boxes in xml
  output_doc = xml.dom.minidom.getDOMImplementation().createDocument(None, "annotations", None)
  node_1 = output_doc.createElement("results")
  node_2 = output_doc.createElement("annotation")
  output_doc.documentElement.appendChild(node_1)
  node_1.appendChild(node_2)
  attribute_names = ["x", "y", "width", "height", "angle", "flip"]
  for oriented_box in oriented_boxes:
    box = output_doc.createElement("oriented_box")
    node_2.appendChild(box)
    for attribute_name, attribute in zip(attribute_names, oriented_box):
      #print "setting attribute " + attribute_name
      box.setAttribute(attribute_name, str(attribute))
    # For convenience, add points describing the four corners of the box, like a polygon
    for corner in getCorners(oriented_box):
      pt = output_doc.createElement("pt")
      pt.setAttribute("x", str(corner[0]))
      pt.setAttribute("y", str(corner[1]))
      box.appendChild(pt)

  return output_doc.toprettyxml()

# A helper function for crop_oriented_boxes()
def create_transform(x, y, angle, flip):
  transform = cv.CreateMat(2,3,cv.CV_32FC1)  # A simple 2x3 matrix of floats
  angle = pi - angle  # Work on the upside-down image
  angle = angle - pi  # We want the hand to look upside down in the upside down image
  c = cos(angle)
  s = sin(angle)
  # Create rotation matrix
  transform[0,0] = c
  transform[1,0] = s
  transform[0,1] = -s
  transform[1,1] = c
  transform[0,2] = x
  transform[1,2] = y
  if flip:
    transform[0,0] = -c  # Flip the x axis of the output image
    transform[1,0] = -s
  #for i in range(2):
  #  for j in range(2):
  #    print transform[i,j]
  return transform


# A generator to read all oriented box descriptions from an xml file
# The expected format is:
# <oriented_box angle="3.09" height="137.75" width="106.65" x="467.43" y="52.61"/>
def read_oriented_boxes(xml_filename):
  input_doc = xml.dom.minidom.parse(xml_filename)
  attribute_names = ["x", "y", "width", "height", "angle", "flip"]
  for box in input_doc.getElementsByTagName("oriented_box"):
    oriented_box = []
    for attribute_name in attribute_names:
      oriented_box.append(float(box.getAttribute(attribute_name)))
    yield oriented_box

# Takes a [x,y,w,h,angle,flip] oriented box, and an OpenCV image, and returns
# the cropped box as an OpenCV image
def crop_single_oriented_box(oriented_box, image):
  x, y, width, height, angle, flip = oriented_box
  width = int(ceil(width))
  height = int(ceil(height))
  # GetQuadrangleSubPix uses coordinates for the destination image that are
  # centered in the middle of the image.
  cropped = cv.CreateImage([width, height], image.depth, image.nChannels)
  transform = create_transform(x, y, angle, flip)
  cv.GetQuadrangleSubPix(image, cropped, transform)
  return cropped


# A generator returning OpenCV images cropped from the image provided
# in argument. The xml file contains descriptions of oriented boxes.
def cropped_oriented_boxes(xml_filename, image_filename):
  image = cv.LoadImage(image_filename)
  for oriented_box in read_oriented_boxes(xml_filename):
    yield crop_single_oriented_box(oriented_box, image)

# Determines whether two oriented boxes overlap
def oriented_boxes_overlap(box1, box2):
  def a_and_b_are_separated_by_an_edge_of_b(a, b):
    x, y, width, height, angle, flip = b
    center = array([x, y], dtype=float)
    orientations = [angle, angle+pi/2, angle+pi, angle-pi/2]
    distances = [width/2, height/2, width/2, height/2]
    for orientation, distance in zip(orientations, distances):
      for corner_tuple in getCorners(a):
        corner = array(corner_tuple)  # convert to array
        edge_normal = array([cos(angle), sin(angle)])
        if dot_product(corner-center, edge_normal) < distance:
          # This edge is not a separating line
          break
      else:
        # This edge is a separating line ==> the two boxes do not overlap
        return True
    # No edge of b is a separating line
    return False
  separated = a_and_b_are_separated_by_an_edge_of_b(box1, box2) or \
              a_and_b_are_separated_by_an_edge_of_b(box2, box1)
  return not separated

# A generator returning OpenCV images cropped from the image provided
# in argument, at random locations and orientations that do *not* overlap with
# the oriented boxes given in the xml file, but with the same size.
# The purpose of this function is to create negative training sets.
# It is unnecessarily slow, but it is usually run only once anyway.
def negative_cropped_oriented_boxes(xml_filename, image_filename):
  image = cv.LoadImage(image_filename)
  size = cv.GetSize(image)
  # Transform the generator to a list, to avoid reparsing the xml file
  oriented_boxes = list(read_oriented_boxes(xml_filename))
  taboo_list = list(oriented_boxes)
  for i in range(3):
    for box in oriented_boxes:
      x, y, width, height, angle, flip = box
      for trial in range(10):
        # Propose a negative box (it can extend outside the image a little)
        n_angle = random.uniform(-pi,pi)
        n_x = random.uniform(width/2, size[0]-1-width/2)
        n_y = random.uniform(height/2, size[1]-1-height/2)
        n_flip = random.choice([0,1])
        candidate = [n_x, n_y, width, height, n_angle, n_flip]
        for b in taboo_list:
          if oriented_boxes_overlap(b, candidate):
            # Candidate is invalid. Try another random location
            break
        else:
          # No overlap with any other box, this is a valid negative example
          taboo_list.append(candidate)  # We dont want to resample the same
          yield crop_single_oriented_box(candidate, image)
          # Exit the rejection sampling loop and move on to the next box
          break
      

#----------------------------------------------------------------------
#                   Test functions (for debugging)
#----------------------------------------------------------------------

def visual_test():
  import Tkinter

  # Can add other examples:
  inputs = [[(140,100), (100,140), (60,150), (40,130), (20,70), (40,20), (90,30)]]
  for input in inputs:
    points = []
    for P in input:
      points.append(array(P, dtype=float))
    bbox = single_hand_outline_to_oriented_box(points, 0.1, 1.0)
    
    print bbox
    bbox_polygon = getCorners(bbox)
    root = Tkinter.Tk()
    C = Tkinter.Canvas(root, width=500, height=500)
    C.pack()
    C.create_polygon(bbox_polygon, fill="blue")
    #print bbox_polygon
    C.create_polygon(input, fill="red")
    C.create_line(input[5], input[6], width=3)
  root.mainloop()
  
