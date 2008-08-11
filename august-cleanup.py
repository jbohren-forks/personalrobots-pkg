#!/usr/env python

import sys
import os

SVN_REV_MIN = 2785
PR_URL = 'https://personalrobots.svn.sf.net/svnroot/personalrobots/pkg/trunk'

## ros package path
rpp = os.environ['ROS_PACKAGE_PATH']

# directories to purge
dirs = [
    '3rdparty/drivers',
    '3rdparty/gazebo_git',    
    'messages',
    'robot_models',
    'services',    
    'simulator/tools',
    'unported/vision',
    'util/features', 
    'util/gui', 
    'util/scan_utils',
    'util/transforms',
    ]

# get the svn info for this dir

from subprocess import Popen, PIPE

svninfo = (Popen(['svn', 'info'], stdout=PIPE).communicate()[0] or '').strip().split('\n')

# verify the current working directory is the same as personal robots

import string
svn_url = None
for l in svninfo:
    vals = l.split(':')
    if vals[0] == 'URL':
        svn_url = ':'.join(vals[1:]).strip()
        print svn_url

        break
else:
    print >> sys.stderr, "ERROR: unable to determine current SVN URL"
    sys.exit(1)
if svn_url != PR_URL:
    
    #there are various setups in which this might be true, so check
    #against RPP instead, which isn't as strong
    
    print >> sys.stderr, "WARNING: svn info doesn't match, validating against ROS_PACKAGE_PATH instead"

    for d in rpp.split(':'):
        if os.path.samefile(d, '.'):
            rpp = d
            break
    else:
        print >> sys.stderr, "ERROR: this must be run from the personal robots directory"
        sys.exit(1)
else:
    print "... SVN URL matches personalrobots"
    
print "... validated current working directory"

# verify the subversion rev number for this script

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
    time.sleep(2.0)
    shutil.rmtree(p)

print "... CLEAN!"
