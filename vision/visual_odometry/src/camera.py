import numpy
from math import *

import Image as Image
import ImageDraw as ImageDraw

# Fx, Fy, Tx, Clx, Crx, Cy
class Camera:
  def __init__(self, params):
    self.params = params
    (Fx, Fy, Tx, Clx, Crx, Cy) = params

    # Left Projection matrix
    self.Pl = numpy.array([
      [ Fx, 0,  Clx, 0 ],
      [ 0,  Fy, Cy,  0 ],
      [ 0,  0,  1,   0 ]
    ])

    # Right Projection matrix
    self.Pr = numpy.array([
      [ Fx, 0,  Crx, -Fx*Tx ],
      [ 0,  Fy, Cy,  0      ],
      [ 0,  0,  1,   0      ]
    ])

  def cam2pixLR(self, X, Y, Z):
    """ takes camera space (X,Y,Z) and returns the pixel space (x,y,d) """
    def xform(P, pt):
      (x,y,w) = numpy.dot(P, pt).transpose()[0]
      return (x/w, y/w)
    campt = numpy.array([ [X], [Y], [Z], [1] ])
    (xl,yl) = xform(self.Pl, campt)
    (xr,yr) = xform(self.Pr, campt)
    assert yl == yr
    return ((xl,yl), (xr,yr))

  def cam2pix(self, X, Y, Z):
    """ takes camera space (X,Y,Z) and returns the pixel space (x,y,d) """
    def xform(P, pt):
      (x,y,w) = numpy.dot(P, pt).transpose()[0]
      return (x/w, y/w)
    campt = numpy.array([ [X], [Y], [Z], [1] ])
    (xl,yl) = xform(self.Pl, campt)
    (xr,yr) = xform(self.Pr, campt)
    assert yl == yr
    return (xl, yl, ((xl - xr)*1.))
    
class VidereCamera(Camera):
  def __init__(self, config_str):
    section = ""
    in_proj = 0
    matrix = []
    for l in config_str.split('\n'):
      if len(l) > 0 and l[0] == '[':
        section = l.strip('[]')
      ws = l.split()
      if ws != []:
        if section == "right camera" and ws[0].isalpha():
          in_proj = (ws[0] == 'proj')
        elif in_proj:
          matrix.append([ float(s) for s in l.split() ])
    Fx = matrix[0][0]
    Fy = matrix[1][1]
    Cx = matrix[0][2]
    Cy = matrix[1][2]
    Tx = -matrix[0][3] / Fx
    Camera.__init__(self, (Fx, Fy, Tx, Cx, Cx, Cy))
