#!/usr/env python

import sys
import os

SVN_REV_MIN = 2751

# directories to purge
dirs = [
    '3rdparty/drivers',
    '3rdparty/gazebo_git',    
    'messages',
    'robot_models',
    'services',    
    'util/gui', 
    'util/scan_utils',
    'util/transforms',
    ]

# verify the current working directory is the same as personal robots
rpp = os.environ['ROS_PACKAGE_PATH']

for d in rpp.split(':'):
    if os.path.samefile(d, '.'):
        rpp = d
        break
else:
    print >> sys.stderr, "This must be run from the personal robots directory"
    sys.exit(1)

print "... validated current working directory"

# verify the subversion rev number for this script
from subprocess import Popen, PIPE

svninfo = (Popen(['svn', 'info'], stdout=PIPE).communicate()[0] or '').strip().split('\n')

import string
svn_rev = None
for l in svninfo:
    vals = l.split(':')
    if vals[0] == 'Revision':
        svn_rev = string.atoi(vals[1])
        break
else:
    print >> sys.stderr, "ERROR: unable to determine current SVN revision"
    sys.exit(1)
if svn_rev < SVN_REV_MIN:
    print >> sys.stderr, "ERROR: your personalrebots repository is out of date. This script is designed for SVN revision number [%s] or later"%SVN_REV_MIN
    sys.exit(1)
else:
    print "... validated SVN revision number (%s >= %s)"%(svn_rev, SVN_REV_MIN)
    
# delete the now dead directories
print "... rub-a-dub-dub, here comes the scrub"
import shutil
import time
for d in dirs:
    p = os.path.join(rpp, d)
    if not os.path.exists(p):
        continue
    print "... deleting [%s]"%p
    shutil.rmtree(p)
    time.sleep(1.0)

print "... CLEAN!"
