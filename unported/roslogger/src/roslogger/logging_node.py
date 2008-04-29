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
logging_node

To run, invoke nodes/logging_node
"""

import os, sys, getopt, traceback, logging
import rospy
import common_flows

NAME = 'logging_node'

def usage(stdout, progname):
    print >>stdout, """%s [-h]

"""%progname+rospy.USAGE_ENV

def logging_nodeMain(argv, stdout, env):
    #check arguments for a help flag
    optlist, args = getopt.getopt(argv[1:], "h?", ["help"])
    if filter(lambda x: x in ["-h","-?","--help"], optlist):
        usage(stdout, argv[0])
        return

    rospy.start() #blocks until connected to master
    master = rospy.getMaster();
    #initialize flows
    in_string_in = common_flows.FlowString(".input")
    while 1: #TODO: replace with shutdown check
        #read/publish flows
        # - read inflow data
        #print master.getParam('.name')
        string_inData = in_string_in.receive()
        print "logging_node recieved: " + string_inData.data
        # - create outflow data objects
        # - publish outflow data
        
if __name__ == '__main__':
    try:
        logging_nodeMain(sys.argv, sys.stdout, os.environ)
    except Exception, e:
        traceback.print_exc()
        #attempt to log, may fail if logger is not properly initialized
        logger = logging.getLogger(NAME)
        logger.error(str(e)+"\n"+traceback.format_exc())        
        print "Exception is causing %s exit, check log for details"%NAME

    print "exiting"

        
