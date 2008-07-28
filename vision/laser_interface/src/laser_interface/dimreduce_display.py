import laser_detector as ld
import opencv as cv
import opencv.highgui as hg
import util as ut
import dimreduce as dr
import numpy as np

dataset = ld.load_pickle('PatchClassifier.dataset.pickle')
hg.cvNamedWindow('image', 1)
hg.cvNamedWindow('context', 1)

def reconstruct_input(nparray):
    #split
    npbig   = nparray[0:675].A
    npsmall = nparray[675:].A

    #reshape
    npbig_rs = npbig.reshape((15,15,3))
    npsml_rs = npsmall.reshape((3,3,3))

    #convert to cv
    img     = ut.np2cv(npbig_rs.astype(np.uint8))
    context = ut.np2cv(npsml_rs.astype(np.uint8))
    return img, context

def normalize_for_display(vec):
    return (255 * ((vec - np.min(vec)) / np.max(vec)))

def display(vec, name):
    patch, context = reconstruct_input(vec)
    hg.cvSaveImage(name + '_patch.png', patch)
    hg.cvSaveImage(name + '_context.png', context)
    hg.cvShowImage('image', patch)
    hg.cvShowImage('context', context)
    hg.cvWaitKey()

pca_basis = normalize_for_display(dataset.projection_basis)
for i in range(pca_basis.shape[1]):
    print 'basis', i
    display(pca_basis[:,i], 'pca_basis'+str(i))


















































































#projection_vectors = dr.pca_vectors(dataset.inputs, 0.95)
#projection_vectors = normalize_for_display(projection_vectors)
#print 'number of projection bases', projection_vectors.shape

#image = hg.cvLoadImage('1frame489.png')
#subrect = cv.cvCloneImage(cv.cvGetSubRect(image, cv.cvRect(40, 40, 20, 30)))
#hg.cvShowImage('image', image)
#hg.cvWaitKey()
#
#hg.cvShowImage('image', subrect)
#print 'original'
#hg.cvWaitKey()
#
#npsub   = ut.cv2np(subrect, 'BGR')
#img     = ut.np2cv(npsub.reshape((30,20,3)))
#hg.cvShowImage('image', img)
#print 'reconstructed'
#hg.cvWaitKey()

#for i in range(dataset.num_examples()):
#    patch, context = reconstruct_input(dataset.inputs[:,i])
#    hg.cvShowImage('image', patch)
#    hg.cvShowImage('context', context)
#    hg.cvWaitKey(33)































#PKG = 'laser_interface'
#import sys, os, subprocess
#try:
#    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
#    sys.path.append(os.path.join(rostoolsDir,'src'))
#    import rostools.launcher
#    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
#except ImportError:
#    print >> sys.stderr, "\nERROR: Cannot locate rostools"
#    sys.exit(1)  
#
##from pkg import *
#import rospy
#from std_msgs.msg import Point3DFloat64
#from std_msgs.msg import RobotBase2DOdom
#import sys
#
#def debug_me(p):
#    print 'received', p.pos.x, p.pos.y, p.pos.th
#rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
#rospy.ready('test')
#rospy.spin()
#def swap_br(npimage):
#    b = npimage[:,:,0].copy()
#    r = npimage[:,:,2].copy()
#    npimage[:,:,0] = r
#    npimage[:,:,2] = b
#    return npimage
#
#image   = hg.cvLoadImage('1frame489.png')
#npimg   = ut.cv2np(image)
#swapped = swap_br(npimg.copy())
#cvimg   = ut.np2cv(swapped)
#
#hg.cvNamedWindow('hai', 1)
#hg.cvShowImage('hai', cvimg)
#hg.cvWaitKey(10)
#
#





















