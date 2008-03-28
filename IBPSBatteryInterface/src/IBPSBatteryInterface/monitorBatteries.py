# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
"""
monitorBatteries

To run, invoke nodes/monitorBatteries
"""
 
import os, sys, getopt, traceback, logging
import rospy
import time
import IBPSBatteryInterface.parseIBPS 

NAME = 'monitorBatteries'

def usage(stdout, progname):
    print >>stdout, """%s [-h]

"""%progname+rospy.USAGE_ENV

def monitorBatteriesMain(argv, stdout, env):
    # default arguments
    server = "http://localhost"
    server_port = rospy.DEFAULT_TEST_PORT

    #check arguments for a help flag
    optlist, args = getopt.getopt(argv[1:], "h?p:s:", ["help","port=","server=","test"])
    for o, a in optlist:
        if o in ("-h","-?","--help"):
            usage(stdout, argv[0])
            return
        elif o in ("--test"):
            server_port = rospy.DEFAULT_TEST_PORT
        elif o in ("-p", "--port"):
            server_port = a
        elif o in ("-s", "--server"):
            server = a
            
    serverUri = '%s:%s/'%(server,server_port)
    print "Looking for server at %s"%serverUri
    os.environ[rospy.ROS_MASTER_URI] = serverUri
    os.environ[rospy.ROS_NODE] = "viewGraph"
    os.environ[rospy.ROS_PORT] = str(0) # any


    master = rospy.getMaster()


    IBPSBatteryInterface.parseIBPS.main()
        
if __name__ == '__main__':
    try:
        monitorBatteriesMain(sys.argv, sys.stdout, os.environ)
    except Exception, e:
        traceback.print_exc()
        #attempt to log, may fail if logger is not properly initialized
        logger = logging.getLogger(NAME)
        logger.error(str(e)+"\n"+traceback.format_exc())        
        print "Exception is causing %s exit, check log for details"%NAME

    print "exiting"

        
