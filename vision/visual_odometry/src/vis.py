import sys, os
import time
import PIL.Image

import votools as VO

class Vis:

  def __init__(self):
    self.iw = VO.imWindow()

  def show(self, image, pts):
    self.iw.DisplayImage(image, pts)
