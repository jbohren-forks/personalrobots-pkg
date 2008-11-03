import rostools
rostools.update_path('visual_odometry')
import rostest

import sys
sys.path.append('lib')

import visual_odometry as VO
import rosrecord
import rospy

import Image as Image

import camera
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, FeatureDetectorFast, FeatureDetectorStar, FeatureDetectorHarris, DescriptorSchemeCalonder, DescriptorSchemeSAD

import numpy
import pylab

def load_from_bag(filename, selected_frames):
  cam = None
  framecounter = 0
  afs = {}
  for topic, msg in rosrecord.logplayer(filename):
    if rospy.is_shutdown():
      break

    if topic == "/videre/cal_params" and not cam:
      cam = camera.VidereCamera(msg.data)

    if cam and topic == "/videre/images":
      print "frame", framecounter
      if framecounter in selected_frames:
        def imgAdapted(msg_img):
          return Image.fromstring("L", (msg_img.width, msg_img.height), msg_img.data)
        imgR = imgAdapted(msg.images[0])
        imgL = imgAdapted(msg.images[1])
        afs[framecounter] = SparseStereoFrame(imgL, imgR)
        if framecounter == max(selected_frames):
          break
      framecounter += 1
  return (cam, afs)

def load_from_dir(dir, selected_frames):
  afs = dict([(f, SparseStereoFrame(Image.open("%s/left-%04d.ppm" % (dir,f)), Image.open("%s/right-%04d.ppm" % (dir,f)))) for f in selected_frames])
  return afs

########################################################################

# Example 1: load explicit keyframes from sequence in /u/konolige/vslam/data/indoor1/:

cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
afs = load_from_dir("/u/konolige/vslam/data/indoor1/", [0, 39, 43, 45, 48, 52, 54, 56, 62, 64, 67, 70, 73, 74, 77, 78, 80, 82, 83, 84, 85, 87, 92, 97, 99, 102, 106, 109, 110, 113, 120, 124, 126, 129, 133, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 154, 157, 160, 163, 164, 168, 171, 173, 175, 178, 180, 183, 187, 190, 194, 201, 204, 213, 220, 223, 224, 225, 227, 229, 232, 235, 237, 241, 245, 246, 248, 250, 251, 252, 253, 254, 256, 257, 258, 259, 260, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 275, 280, 286, 292, 297, 298, 300, 305, 310, 317, 323, 327, 333, 337, 340, 346, 354, 362, 365, 368, 372, 376, 381, 386, 389, 393, 397, 401, 403, 405, 408, 411, 412, 413, 417, 420, 422, 424, 444, 449, 452, 456, 457, 459, 462, 465, 466, 467, 471, 472, 473, 475, 476, 478, 480, 481, 482, 484, 485, 486, 487, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 501, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 556, 557, 559, 561, 562, 563, 565, 566, 569, 572, 574, 576, 580, 581, 584, 585, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 600, 603, 604, 607, 612, 614, 619, 624, 625, 630, 634, 638, 641, 645, 647, 649, 650, 651, 652, 653, 654, 655, 658, 662, 664, 666, 669, 672, 674, 676, 678, 682, 686, 689, 692, 696, 698, 699, 702, 704, 705, 708, 710, 712, 714, 718, 722, 724, 731, 737, 743, 748, 753, 754, 756, 757, 760, 763, 765, 767, 768, 770, 771, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 788, 792, 798, 802, 804, 805, 807, 808, 809, 810, 811, 812, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 827, 831, 838, 843, 845, 850, 855, 858, 859, 861, 862, 863, 864, 866, 867, 868, 870, 871, 872, 874, 876, 877, 878, 879, 881, 884, 888, 891, 892, 893, 894, 895, 897, 898, 901, 905, 909, 914, 915, 917, 920, 926, 932, 936, 942, 947, 951, 952, 953, 954, 955, 956, 957, 958, 959, 960, 961, 962, 963, 964, 965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979, 980, 981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995, 996, 997, 998, 999, 1000, 1001, 1002, 1003, 1004, 1005, 1007, 1008, 1010, 1012, 1013, 1016, 1019, 1022, 1025, 1026, 1028, 1029, 1032, 1033, 1035, 1037, 1039, 1041, 1043, 1045, 1048, 1049, 1050, 1051, 1053, 1055, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1068, 1071, 1077, 1081, 1088, 1091, 1094, 1099, 1103, 1107, 1113, 1118, 1124, 1128, 1131, 1134, 1135, 1137, 1139, 1142, 1145, 1146, 1147, 1148, 1149, 1150, 1151, 1152, 1155, 1156, 1158, 1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1178, 1179, 1180, 1182, 1185, 1188, 1192, 1198, 1202, 1207, 1211, 1215, 1216, 1221, 1225, 1227, 1233, 1237, 1244, 1250, 1258, 1263, 1269, 1274, 1280, 1285, 1291, 1298, 1305, 1308, 1313, 1317, 1320, 1323, 1325, 1328, 1329, 1332, 1334, 1335, 1337, 1339, 1340, 1341, 1342, 1343, 1344, 1345, 1346, 1347, 1348, 1349, 1350, 1352, 1354, 1355, 1356, 1357, 1358, 1359, 1360, 1361, 1362, 1363, 1367, 1372, 1377, 1379, 1381, 1382, 1383, 1384, 1385, 1386, 1388, 1391, 1394, 1395, 1399, 1404, 1411, 1418, 1421, 1423, 1429, 1432, 1436, 1438, 1440, 1444, 1447, 1455][::100])

# Example 2: load frames 560,570... 940 from loop1-mono.bag

# (cam, afs) = load_from_bag( "/u/prdata/videre-bags/loop1-mono.bag", range(560, 941, 100))

vos = [
VisualOdometer(cam, feature_detector = FeatureDetectorStar(), descriptor_scheme = DescriptorSchemeCalonder()),
VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeCalonder()),
VisualOdometer(cam, feature_detector = FeatureDetectorHarris(), descriptor_scheme = DescriptorSchemeCalonder())
]

def metric(af0, af1):
  """ Given a pair of frames, return the distance metric using inliers from the solved pose """
  pairs = vo.temporal_match(af0, af1)
  if len(pairs) > 10:
    (inl, rot, shift) = vo.solve(af0.kp, af1.kp, pairs)
    if inl == 0:
      return 0
    else:
      return min(200, inl)
  else:
    return 0

  
fig = pylab.figure()
img1_ax = fig.add_subplot(222)
img2_ax = fig.add_subplot(224)

def onclick(event):
    index1 = int(event.ydata)
    index2 = int(event.xdata)
    display_images(index1, index2)

for (mode_i, vo) in enumerate(vos):
  keys = sorted(afs.keys())

  for f in keys:
    vo.find_keypoints(afs[f])
    vo.find_disparities(afs[f])
    vo.collect_descriptors(afs[f])

  results = []
  for i in keys:
    line = [ metric(afs[i], afs[j]) for j in keys if j <= i ] + ([0] * len([j for j in keys if j > i]))
    results.append(line)

  pylab.figure(2 + mode_i)
  pylab.pcolor(numpy.array(results))
  pylab.colorbar()
  pylab.connect('button_press_event', onclick)
  pylab.title("%s %s" % (vo.feature_detector.name(), vo.descriptor_scheme.name()))

  def display_images(index1, index2):
      pylab.figure(1)
      for ax,index in ([img1_ax, index1], [img2_ax, index2]):
        ax.imshow(pylab.asarray(afs[keys[index]].lf), cmap=pylab.cm.gray, interpolation='nearest')
        ax.set_title('Frame %d' % keys[index])
      pylab.draw()

pylab.show()
