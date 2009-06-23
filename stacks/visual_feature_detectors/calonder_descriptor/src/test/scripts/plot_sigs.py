#!/usr/bin/python
from pylab import *
import sys

# load the data
sigs = load(sys.argv[1])
classes = len(sigs)
sig_length = len(sigs[0])
x = range(1, sig_length + 1)

fig = figure()
ax = fig.add_subplot(111)
index = 0

def plot_response(index):
    posterior = sigs[index,:]
    ax.hold(False)
    markers = ax.stem(x, posterior)[0]
    markers.set_picker(5)
    #ax.hold(True)
    #ax.stem([index + 1], [posterior[index]], linefmt='r-', markerfmt='ro')
    #ax.set_title('Class %i' % (index + 1))
    ax.set_title('Class %i' % index)
    #ax.set_xlabel('learned classes')
    #ax.set_ylabel('classifier response')
    draw()
plot_response(index)

def next_response(direction):
    global index
    index += direction
    if index < 0: index = 0
    if index >= classes: index = classes - 1
    plot_response(index)

def keypress(event):
    if event.key in ('n', 'N'): next_response(1)
    elif event.key in ('p', 'P'): next_response(-1)

def onpick(event):
    global index
    index = event.ind[0]
    plot_response(index)

connect('key_press_event', keypress)
connect('pick_event', onpick)

show()
