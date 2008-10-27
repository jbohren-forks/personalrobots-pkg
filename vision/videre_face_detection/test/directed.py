import rostools
rostools.update_path('videre_face_detection')
import rostest
import videre_face_detection

import sys
sys.path.append('lib')

import Image
import ImageChops
import ImageDraw

import random
import time
import math

import os

p = videre_face_detection.people()
print p
for (i, f) in enumerate(range(200,240)):
  im = Image.open("/u/jamesb/Desktop/wheelchair640l/%06d.png" % f).convert("L")
  cascade_file = "cascades/haarcascade_frontalface_alt.xml"
  assert os.access(cascade_file, os.R_OK)
  faces = p.detectAllFaces(im.tostring(), im.size[0], im.size[1], cascade_file, 1.0, None, None, True)

  draw = ImageDraw.Draw(im)
  for (x,y,w,h) in faces:
    draw.line((x,y,x+w,y), fill=255)
    draw.line((x,y+h,x+w,y+h), fill=255)
    draw.line((x,y,x,y+h), fill=255)
    draw.line((x+w,y,x+w,y+h), fill=255)
  im.save("out%06d.tiff" % i)
