from pkg import *
from std_msgs.msg import Point3DFloat64
import sys, time
import opencv as cv
import opencv.highgui as hg
import camera as cam
import util as ut
import random_forest as rf
from laser_detector import *
from threading import RLock

def show_processed(image, masks, detection, blobs, detector):
    masker            = Mask(image)
    splitter          = SplitColors(image)
    r, g, b           = splitter.split(image)
    thresholded_image = masker.mask(masks[0], r, g, b)
    draw_detection(thresholded_image, detection)
    hg.cvShowImage('thresholded', thresholded_image)

    draw_detection(image, detection)
    draw_blobs(image, blobs)

    make_visible_binary_image(masks[0])
    draw_detection(masks[0], detection)
    make_visible_binary_image(masks[1])
    make_visible_binary_image(masks[2])

    hg.cvShowImage("video",       image)
    hg.cvShowImage('motion',      masks[1])
    hg.cvShowImage('intensity',   masks[2])

    #key = hg.cvWaitKey(10)
    #if detector != None:
    #    if key == 'T': #down
    #        detector.intensity_filter.thres_high = detector.intensity_filter.thres_high - 5
    #        print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
    #    if key == 'R':
    #        detector.intensity_filter.thres_high = detector.intensity_filter.thres_high + 5
    #        print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
    #    if key == ' ':
    #        hg.cvWaitKey()

def confirmation_prompt(confirm_phrase):
    print confirm_phrase
    print 'y(es)/n(no)'
    k = hg.cvWaitKey()
    if k == 'y':
        return True
    else:
        return False

def append_examples_to_file(dataset, file = PatchClassifier.DEFAULT_DATASET_FILE):
    try:
        loaded_set = load_pickle(file)
        dataset.append(loaded_set)
    except IOError:
        pass
    dump_pickle(dataset, file)

class GatherExamples:
    def __init__(self, hardware_camera, type = 1):
        frames         = hardware_camera.next()
        self.detector  = LaserPointerDetector(frames[0], exposure=exposure, 
                                         dataset=PatchClassifier.DEFAULT_DATASET_FILE,
                                         use_color=False, use_learning=False)
        self.detector2 = LaserPointerDetector(frames[1], exposure=exposure, 
                                         dataset=PatchClassifier.DEFAULT_DATASET_FILE,
                                         use_color=False, use_learning=False)
        for i in xrange(10):
            frames = hardware_camera.next()
            self.detector.detect(frames[0])
            self.detector2.detect(frames[1])
        self.examples = []
        self.type     = type

    def run(self, images, display=True, verbose=False, debug=False):
        image                 = None
        combined              = None
        motion                = None
        intensity             = None
        laser_blob            = None
        intensity_motion_blob = None

        for raw_image, detect in zip(frames, [self.detector, self.detector2]):
            #before = time.time()
            image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
            #diff = time.time() - before
            #print 'took %.2f seconds to run or %.2f fps' % (diff, 1.0/diff)
            if laser_blob != None:
                instance = blob_to_input_instance(image, laser_blob)
                if instance is not None:
                    self.examples.append(instance)
                    print 'got', len(self.examples), 'instances'
            motion, intensity = detect.get_motion_intensity_images()

        if display:
            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, self.detector2)


    def write(self):
        append_examples_to_file(
                matrix_to_dataset(
                    ut.list_mat_to_mat(
                        self.examples, axis=1), type=self.type))

class DetectState:
    def __init__(self, geometric_camera, hardware_camera):
        self.stereo_cam     = geometric_camera
        frames              = hardware_camera.next()
        self.detector       = LaserPointerDetector(frames[0], LaserPointerDetector.SUN_EXPOSURE, 
                                                   use_color=False, use_learning=True)
        self.detector_right = LaserPointerDetector(frames[1], LaserPointerDetector.SUN_EXPOSURE, 
                                                   use_color=False, use_learning=True, 
                                                   classifier=self.detector.classifier)

        for i in xrange(10):
            frames = hardware_camera.next()
            self.detector.detect(frames[0])
            self.detector_right.detect(frames[1])

    def run(self, images, display=True, verbose=False, debug=False):
        l, r = images

        #Detect right image
        _, _, right_cam_detection, stats = self.detector_right.detect(r, verbose=verbose)
        if debug:
            draw_blobs(r, stats)
            draw_detection(r, right_cam_detection)
            hg.cvShowImage('right', r)

        #Detect left image
        image, combined, left_cam_detection, stats = self.detector.detect(l, verbose=verbose)
        if debug: 
            motion, intensity = self.detector.get_motion_intensity_images()
            show_processed(l, [combined, motion, intensity], left_cam_detection, stats, self.detector)
        elif display:
            draw_detection(l, left_cam_detection)
            hg.cvShowImage('video', l)

        #Triangulate
        if right_cam_detection != None and left_cam_detection != None:
            x  = np.matrix(left_cam_detection['centroid']).T
            xp = np.matrix(right_cam_detection['centroid']).T
            result = self.stereo_cam.triangulate_3d(x, xp)
            print '3D point located at', result['point'].T, 
            print 'distance %.2f error %.3f' % (np.linalg.norm(result['point']),  result['error'])
            return result
        else:
            return None

class LaserPointerDetectorNode:
    GATHER_POSITIVE_EXAMPLES = 'GATHER_POSITIVE_EXAMPLES'
    GATHER_NEGATIVE_EXAMPLES = 'GATHER_NEGATIVE_EXAMPLES'
    DETECT                   = 'DETECT'
    CURSOR_TOPIC             = 'cursor3d'

    def __init__(self, exposure = LaserPointerDetector.SUN_EXPOSURE, video = None):
        if video is None:
            #video    = cam.VidereStereo(0, gain=96, exposure=exposure)
            self.video    = cam.StereoFile('measuring_tape_red_left.avi','measuring_tape_red_right.avi')
        else:
            self.video = video
        self.video_lock       = RLock()
        self.camera_model     = cam.KNOWN_CAMERAS['videre_stereo2']
        self.state            = None
        self.state_object     = None
        self.set_state(self.DETECT)
        self.display = True
        self.verbose = False
        self.debug   = False
        self._make_windows()

        #Publish
        self.topic = rospy.TopicPub(self.CURSOR_TOPIC, Point3DFloat64)

        #Ready
        rospy.ready(sys.argv[0])

        print '======================================================'
        print '= Usage'
        print '======================================================'
        print """
                if k == 'p':
                    self.set_state(self.GATHER_POSITIVE_EXAMPLES)
                if k == 'n':
                    self.set_state(self.GATHER_NEGATIVE_EXAMPLES)
                if k == ' ':
                    self.set_state(self.DETECT)
                if k == 'd':
                    self.display = not self.display
                if k == 'v':
                    self.verbose = not self.verbose
                if k == 'g':
                    self.debug   = not self.debug"""

    def _make_windows(self):
        windows = ['video', 'right', 'thresholded', 'motion', 'intensity']
        for n in windows:
            hg.cvNamedWindow(n, 1)
        hg.cvMoveWindow("video",       0,   0)
        hg.cvMoveWindow("right",       800, 0)
        hg.cvMoveWindow("thresholded", 800, 0)
        hg.cvMoveWindow("intensity",   0,   600)
        hg.cvMoveWindow("motion",      800, 600)

    def run(self):
        count = 0
        try:
            while not rospy.isShutdown():
                self.video_lock.acquire()
                start_time = time.time()
                frames = self.video.next()
                result = self.state_object.run(frames, display=self.display, verbose=self.verbose, debug=self.debug)
                self.video_lock.release()
                if result != None:
                    p = result['point']
                    self.topic.publish(Point3DFloat64(p[0,0], p[1,0], p[2,0]))

                if count % (2*30) == 0:
                    diff = time.time() - start_time
                    print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)
                count = count + 1
                k = hg.cvWaitKey(10)
                if k == 'p':
                    self.set_state(self.GATHER_POSITIVE_EXAMPLES)
                if k == 'n':
                    self.set_state(self.GATHER_NEGATIVE_EXAMPLES)
                if k == ' ':
                    self.set_state(self.DETECT)
                if k == 'd':
                    self.display = not self.display
                if k == 'v':
                    self.verbose = not self.verbose
                if k == 'g':
                    self.debug   = not self.debug

        except StopIteration, e:
            pass


    def set_state(self, new_state):
        self.video_lock.acquire()
        print '======================================================'
        print '=  Changing Mode to', new_state
        print '======================================================'
        if self.state_object.__class__ == GatherExamples:
            self.state_object.write()

        if new_state == self.GATHER_POSITIVE_EXAMPLES:
            self.state_object = GatherExamples(self.video, type = 1)

        if new_state == self.GATHER_NEGATIVE_EXAMPLES:
            self.state_object = GatherExamples(self.video, type = 0)

        if new_state == self.DETECT:
            self.state_object = DetectState(self.camera_model, self.video)
        self.state = new_state
        self.video_lock.release()


if __name__ == '__main__':
    if sys.argv[1] == 'sun':
        exposure = LaserPointerDetector.SUN_EXPOSURE
    else:
        exposure = LaserPointerDetector.NO_SUN_EXPOSURE
    LaserPointerDetectorNode(exposure = exposure).run()







































#class GatherNegativeExamplesState:
#    def __init__(self, frames):
#        self.detector  = LaserPointerDetector(frames[0], exposure=exposure, 
#                                        dataset=PatchClassifier.DEFAULT_DATASET_FILE,
#                                        use_color=False, use_learning=False)
#        self.detector2 = LaserPointerDetector(frames[1], exposure=exposure, 
#                                        dataset=PatchClassifier.DEFAULT_DATASET_FILE,
#                                        use_color=False, use_learning=False)
#
#        for i in xrange(10):
#            frames = hardware_camera.next()
#            self.detector.detect(frames[0])
#            self.detector2.detect(frames[1])
#
#        self.negative_examples_for_classifier = []
#
#    def gather(self, frames, display=True):
#        image                 = None
#        combined              = None
#        motion                = None
#        intensity             = None
#        laser_blob            = None
#        intensity_motion_blob = None
#        for raw_image, detect in zip(frames, [self.detector, self.detector2]):
#            image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
#            if laser_blob != None:
#                instance = blob_to_input_instance(image, laser_blob)
#                if instance is not None:
#                    negative_examples_for_classifier.append(instance)
#                    print 'got', len(negative_examples_for_classifier), 'instances'
#            motion, intensity = detect.get_motion_intensity_images()
#
#        if display:
#            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, self.detector2)
#
#    def write(self):
#        append_examples_to_file(
#                matrix_to_dataset(
#                    ut.list_mat_to_mat(
#                        self.negative_examples_for_classifier, axis=1)))








        #l = stereo_cam.camera_left.undistort_img(l)
        #r = stereo_cam.camera_right.undistort_img(r)
        #cv.cvCopy(l, lt)
        #cv.cvCopy(r, rt)
        #l = lt
        #r = rt
        #undistort_time = time.time()
        #triangulation_time = time.time()
        #detect_time = time.time()
        #diff = time.time() - start_time
        #print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)
        #print '   undistort     took %.4f s' % (undistort_time - start_time)
        #print '   detection     took %.4f s' % (detect_time - undistort_time)
        #print '   triangulation took %.4f s' % (triangulation_time - detect_time)



#def learn_run(exposure = LaserPointerDetector.SUN_EXPOSURE, num_examples_to_collect=200, display_during_run = True):
#    hg.cvNamedWindow("video",       1)
#    hg.cvNamedWindow("thresholded", 1)
#    hg.cvNamedWindow('motion',      1)
#    hg.cvNamedWindow('intensity',   1)
#
#    hg.cvMoveWindow("video",       0,   0)
#    hg.cvMoveWindow("thresholded", 800, 0)
#
#    hg.cvMoveWindow("intensity",   0,   600)
#    hg.cvMoveWindow("motion",      800, 600)
#    video     = cam.VidereStereo(0, gain=96, exposure=exposure)

#if display:
#    hg.cvNamedWindow("video", 1)
#    hg.cvMoveWindow("video",   0,   0)
#if debug:
#    hg.cvNamedWindow('right',       1)
#    hg.cvMoveWindow("right", 800,   0)
#    hg.cvNamedWindow("thresholded", 1)
#    hg.cvNamedWindow('motion',      1)
#    hg.cvNamedWindow('intensity',   1)
#    hg.cvMoveWindow("thresholded", 800, 0)
#    hg.cvMoveWindow("intensity",   0,   600)
#    hg.cvMoveWindow("motion",      800, 600)
#self.video = video
#cam.KNOWN_CAMERAS['videre_stereo2']































##Subscribe
#def callback(msg):
#    print "%s: %s"%(msg.name, msg.msg)
#rospy.TopicSub("chat", Msg, callback)
#

#
##Run
#while not rospy.isShutdown():
#    utter = sys.stdin.readline().strip()
#    if utter:
#        pub.publish(Msg(name, utter))

