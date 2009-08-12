#!/usr/bin/python


"""
usage: %(progname)s annotation.xml

  * Reads xml annotations for an image and extracts the polygons, each polygon outlines a hand
  * Converts the polygons to oriented bounding boxes, where the orientation is given by the axis going from the middle of the wrist to the middle finger.
  * The output format is: x, y, width, height, angle, flip
    where x,y is the center of the box, angle is in radians, and flip is 0 if the thumb is on the left, 1 if it is on the right
  
  
"""
import roslib; roslib.load_manifest('hand_tracker'); roslib.update_path('test_roslaunch')
import unittest
import numpy
import cv
from hand_tracker import oriented_bounding_boxes

#----------------------------------------------------------------------
#                         Test functions
#----------------------------------------------------------------------

class TestHandToBox(unittest.TestCase):
  def test_single_hand_outline_to_oriented_box(self):
    # Can add other examples:
    inputs = [[(140,100), (100,140), (60,150), (40,130), (20,70), (40,20), (90,30)]]
    expected_outputs = [[79.952076677316299, 86.198083067092639, 142.00644025354072, 142.00644025354072, 3.1016139664665032, 1]]
    for input,expected_output in zip(inputs,expected_outputs):
      points = []
      for P in input:
        points.append(numpy.array(P, dtype=float))
      output = oriented_bounding_boxes.single_hand_outline_to_oriented_box(points, 0.1, 1.0)
      for o,e in zip(output,expected_output):
        self.assertAlmostEquals(o, e, 5, "Wrong bounding box for the hand")

  def test_convert_polygons_to_oriented_boxes(self):
    xml_string = oriented_bounding_boxes.convert_polygons_to_oriented_boxes("test/test_case.xml", 0.1, 1.0)
    gold_standard = "".join(open("test/test_case_bbox.xml", "r").readlines())
    self.assertEquals(xml_string, gold_standard)

  def test_crop_oriented_boxes(self):
    box_num = 0
    for cropped_image in oriented_bounding_boxes.cropped_oriented_boxes("test/test_case_bbox.xml", "test/test_case.png"):
      box_num += 1
      gold_standard = cv.LoadImage("test/test_case_%03d.png" % box_num)
      self.assertEquals(cv.GetElemType(cropped_image), cv.GetElemType(gold_standard))
      self.assertEquals(cv.GetSize(cropped_image), cv.GetSize(gold_standard))
      diff = cv.CloneImage(gold_standard)
      cv.AbsDiff(cropped_image, gold_standard, diff)
      self.assertEquals(cv.Sum(diff), (0, 0, 0, 0))

#----------------------------------------------------------------------
#                             Main
#----------------------------------------------------------------------

if __name__ == "__main__":
  import rostest
  rostest.unitrun('hand_tracker', 'test_hand_bbox_calculation', TestHandToBox)

  #Some code used to generate the gold standards:

  #xml_string = oriented_bounding_boxes.convert_polygons_to_oriented_boxes("test_case.xml", 0.1, 1.0)
  #print xml_string

  #data = [(140,100), (100,140), (60,150), (40,130), (20,70), (40,20), (90,30)]
  #points = []
  #for P in data:
  #  points.append(numpy.array(P, dtype=float))
  #output = oriented_bounding_boxes.single_hand_outline_to_oriented_box(points, 0.1, 1.0)
  #print output

  #box_num = 0
  #for cropped_image in oriented_bounding_boxes.cropped_oriented_boxes("test_case_bbox.xml", "test_case.png"):
  #  box_num += 1
  #  cv.NamedWindow("win")
  #  cv.ShowImage("win", cropped_image)
  #  cv.WaitKey()
  #  cv.SaveImage("test_case_%03d.png" % box_num, cropped_image)

