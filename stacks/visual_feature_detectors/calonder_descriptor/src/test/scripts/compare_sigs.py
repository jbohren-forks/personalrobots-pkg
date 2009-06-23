#!/usr/bin/python
from pylab import *
import sys
import Image

# load the data
src_sigs = load(sys.argv[1])
test_sigs = load(sys.argv[2])
patch_dir = sys.argv[3]
matches = load('%s/matches.txt' % patch_dir)
classes = len(src_sigs)
sig_length = len(src_sigs[0])
x = range(0, sig_length)

fig = figure()
src_ax = fig.add_subplot(311)
test_ax = fig.add_subplot(312)
err_ax = fig.add_subplot(313)
#src_ax = fig.add_subplot(321)
#test_ax = fig.add_subplot(323)
#err_ax = fig.add_subplot(325)
#src_patch_ax = fig.add_subplot(322)
#test_patch_ax = fig.add_subplot(324)
#match_patch_ax = fig.add_subplot(326)
#sep = 0.03
#horiz = 1 - 2*sep
#vert = 1 - 
#src_ax = fig.add_axes([0.05, 2.0/3, 0.75, 1.0/3])
#test_ax = fig.add_axes([0.05, 1.0/3, 0.75, 1.0/3])
#err_ax = fig.add_axes([0.05, 0, 0.75, 1.0/3])
#src_patch_ax = fig.add_axes([0.8, 2.0/3, 0.2, 1.0/3])
#test_patch_ax = fig.add_axes([0.8, 1.0/3, 0.2, 1.0/3])
src_ax.hold(False)
test_ax.hold(False)
err_ax.hold(False)
index = 0

def plot_response(index):
    # Plot signatures and error
    src_posterior = src_sigs[index,:]
    markers = src_ax.stem(x, src_posterior)[0]
    markers.set_picker(5)
    src_ax.set_title('Class %i' % index)
    src_ax.set_ylabel('Source')
    test_posterior = test_sigs[index,:]
    markers = test_ax.stem(x, test_posterior)[0]
    markers.set_picker(5)
    test_ax.set_ylabel('Test')
    err_ax.stem(x, abs(src_posterior - test_posterior), linefmt='r-', markerfmt='ro')
    err_ax.set_ylabel('Error')

    # Show patches
    #src_patch = asarray(Image.open('%s/source%i.pgm' % (patch_dir, index)))
    #test_patch = asarray(Image.open('%s/test%i.pgm' % (patch_dir, index)))
    #match_patch = asarray(Image.open('%s/source%i.pgm' % (patch_dir, matches[index,1])))
    #src_patch_ax.imshow(src_patch, cmap=cm.gray, interpolation='nearest')
    #test_patch_ax.imshow(test_patch, cmap=cm.gray, interpolation='nearest')
    #match_patch_ax.imshow(match_patch, cmap=cm.gray, interpolation='nearest')

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
