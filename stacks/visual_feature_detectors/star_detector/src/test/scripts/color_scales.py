#!/usr/bin/python
from pylab import *
import sys
import Image

img = asarray(Image.open(sys.argv[1]))
imshow(img, cmap=cm.jet, interpolation='nearest', origin='upper')
colorbar(ticks=range(1, 8))
if len(sys.argv) > 2:
    x,y = load(sys.argv[2], comments='#', usecols=(0,1), unpack=True)
    scatter(x, y, c='w')
show()
