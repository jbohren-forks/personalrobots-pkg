import sys, os
import time
import PIL.Image

import votools as VO

class Vis:

  def __init__(self, title):
    self.iw = VO.imWindow(title)

  def mouse(self):
    return self.iw.mouse()

  def show(self, image, pts):
    self.iw.DisplayImage(image, pts)
