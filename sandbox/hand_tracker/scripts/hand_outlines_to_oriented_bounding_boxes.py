#!/usr/bin/python

"""
 usage: %(progname)s [<xml_file> | --folder=<folder_name>] [--margin=<margin>] [--aspect_ratio=<ratio>]

 This program converts polygons, in XML format, to corresponding oriented
 bounding boxes, also in XML format.
 When using %(progname)s <image_file>
 The XML files should be in <folder_name>/polygons/ and the results are stored
 in <folder_name>/bbox/. The xml files end in _polygons.xml, which is replaced
 by _bbox.xml.
 The polygons represent outlines of hands, so they have 7 points, representing
 the position of the tips of the fingers starting from the thumb, and the base
 of the hand on each side of the wrist.
 The margin (default 0.1) is how much to expand the bounding box to get context.
 The aspect ratio (default 1.0) is height/width. The box is expanded to achieve
 it.

 Example:
 <?xml version=\"1.0\" ?>
 <annotations>
         <results>
                 <annotation>
                         <polygon name=\"door\" sqn=\"1\">
                                 <pt ct=\"1248131092987\" x=\"324.544\" y=\"127.68\"/>
                                 <pt ct=\"1248131094519\" x=\"350.656\" y=\"65.472\"/>
                                 <pt ct=\"1248131095895\" x=\"374.272\" y=\"49.344\"/>
                                 <pt ct=\"1248131096555\" x=\"384.256\" y=\"54.272\"/>
                                 <pt ct=\"1248131097649\" x=\"405.376\" y=\"69.248\"/>
                                 <pt ct=\"1248131099219\" x=\"392.96\" y=\"140.096\"/>
                                 <pt ct=\"1248131102263\" x=\"364.352\" y=\"157.504\"/>
                         </polygon>
                         <polygon name=\"door\" sqn=\"2\">
                                 <pt ct=\"1248131109098\" x=\"241.216\" y=\"131.392\"/>
                                 <pt ct=\"1248131110890\" x=\"202.688\" y=\"62.976\"/>
                                 <pt ct=\"1248131111924\" x=\"176.576\" y=\"56.768\"/>
                                 <pt ct=\"1248131113644\" x=\"169.152\" y=\"64.256\"/>
                                 <pt ct=\"1248131115870\" x=\"154.176\" y=\"80.384\"/>
                                 <pt ct=\"1248131119816\" x=\"179.072\" y=\"152.512\"/>
                                 <pt ct=\"1248131121170\" x=\"208.896\" y=\"158.72\"/>
                         </polygon>
                  </annotation>
                  <meta load_time=\"1248131087334\" submit_time=\"1248131180210\"/>
         </results>
 </annotations>

 The vertical axis of the bounding box is the line joining the tip of the
 middle finger to the center of the wrist (average of the last two points).
 The output format for the oriented bounding box is:

 (x, y, width, height, angle, flip)
 
 where (x, y) is the middle of the box, expressed in image coordinates.
 flip is 0 if the thumb is on the left, 1 if it is on the right.
 Example:
 <?xml version=\"1.0\" ?>
 <annotations>
         <results>
                 <annotation>
                         <oriented_box angle=\"3.0975413757\" height=\"137.751329989\" width=\"106.658248566\" x=\"467.438913853\" y=\"52.6154809788\">
                                 <pt x=\"411.128444308\" y=\"-13.8449113225\"/>
                                 <pt x=\"517.683223637\" y=\"-18.5418240535\"/>
                                 <pt x=\"523.749383397\" y=\"119.07587328\"/>
                                 <pt x=\"417.194604068\" y=\"123.772786011\"/>
                         </oriented_box>
                         <oriented_box angle=\"2.96727136416\" height=\"135.695919826\" width=\"98.3987823413\" x=\"253.265692251\" y=\"56.7564102996\">
                                 <pt x=\"193.044409957\" y=\"-1.53014831232\"/>
                                 <pt x=\"289.951907695\" y=\"-18.5964088399\"/>
                                 <pt x=\"313.486974544\" y=\"115.042968912\"/>
                                 <pt x=\"216.579476806\" y=\"132.109229439\"/>
                         </oriented_box>
                 </annotation>
         </results>
 </annotations>

"""

import roslib; roslib.load_manifest('hand_tracker')
import sys
import os
import getopt

from hand_tracker import oriented_bounding_boxes

#----------------------------------------------------------------------
#                             Main
#----------------------------------------------------------------------

def usage(progname):
  print __doc__ % vars()


def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "debug", "folder=", "margin=", "aspect_ratio="])
  folder = None
  margin = 0.1
  aspect_ratio = 1.0

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--folder":
      folder = val
    elif field == "--margin":
      margin = val
    elif field == "--aspect_ratio":
      aspect_ratio = val

  if folder:
    polygons_input_folder = os.path.join(folder,"polygons")
    bbox_output_folder = os.path.join(folder,"bbox")
    if not os.path.exists(bbox_output_folder):
      os.mkdir(bbox_output_folder)
    xml_filenames = os.listdir(polygons_input_folder)
  elif (len(args) > 0):
    polygons_input_folder = ""
    bbox_output_folder = ""
    xml_filenames = args
  else:
    usage(progname)
    return
  

  for xml_input_filename in xml_filenames:
    if (len(xml_input_filename) >= 13):
      full_xml_input_filename = os.path.join(polygons_input_folder, xml_input_filename)
      xml_string = oriented_bounding_boxes.convert_polygons_to_oriented_boxes(
        full_xml_input_filename, margin, aspect_ratio)
      # Replace the _polygons.xml extension with a _bbox extension.xml
      xml_output_filename = xml_input_filename[0:-13] + "_bbox.xml"
      xml_output_file = open(os.path.join(bbox_output_folder, xml_output_filename), 'w')
      xml_output_file.write(xml_string)
      print "Wrote " + xml_output_filename

        
               

if __name__ == "__main__":
  #oriented_bounding_boxes.visual_test()
  main(sys.argv, sys.stdout, os.environ)
