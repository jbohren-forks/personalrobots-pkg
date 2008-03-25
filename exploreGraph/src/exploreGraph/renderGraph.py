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
renderGraph

To run, invoke nodes/renderGraph
"""

import os, sys, getopt, traceback, logging
import rospy
import time
from rospy_demo.util import testMode
import yapgvb, user, operator  # for yapgvb that's not above
import Image
from pyImageViewer.pyImageString import *

NAME = 'renderGraph'
sleep_time = 0.1

def require(args):
    """
    simple utility that raises an Exception on API call failure or returns the
    value parameter of the API call.
    @param args: return value from master/slave API call.
    """
    code, msg, val = args
    if code < 1:
        raise Exception("Failure: %s"%msg)
    return val

def usage(stdout, progname):
    print >>stdout, """%s [-h]

"""%progname+rospy.USAGE_ENV

def renderGraphMain(argv, stdout, env):
    #check arguments for a help flag
    optlist, args = getopt.getopt(argv[1:], "h?", ["help","test"])
    for o, a in optlist:
        if o in ("-h","-?","--help"):
            usage(stdout, argv[0])
            return
        if o == "--test":
            testMode(NAME)
            
    master = rospy.getMaster()

    out_FlowImageString = OutflowpyImageString(".imageOut")
    require(master.addMachine('default', rospy.getRosRoot(), 'localhost', 22, '', ''))
    require(master.addNode('','imageViewer','pyImageViewer','imageViewer','default',0))

    rospy.ready()

    require(master.connectFlow(out_FlowImageString.locator, "imageViewer:imageIn",1))
    while not rospy.isShutdown():
        status_code, statusMessage, [nodes,flows] = master.getGraph()
        #        print 'nodes' + str(nodes)
        #        print 'flows ' + str(flows)
        
        print '--------------------------------------------'
        print 'This is the graph:'
        print 'refreshing every '+str(sleep_time)+' seconds'
        print '--------------------------------------------'
        
        for anode in nodes:
            status_code, statusMessage, [machine, address, port] = master.getNodeAddress(anode)
            print 'NODE: ' + str(anode) + ' on ' + str(machine) + ' at ' + str(address) + ' on port: ' + str(port)
            for aflow in flows:
                aflow_split= str(aflow[1]).split('.')
                destination = aflow_split[0]
                aflow_split_source = str(aflow[0]).split('.')
                source = aflow_split_source[0]
                if destination == anode:
                    print '\tINFLOW: ' + str(aflow_split[1]) + ' from: ' + str(aflow[0])
                if source == anode:
                    print '\tOUTFLOW: ' + str(aflow_split_source[1]) + ' to: ' + str(aflow[1])

        output_file = 'ROSGraph.jpeg'
        pid = os.fork()  # This is a hack to get around an underlying memory leak in
        # the agraph library.  
        if pid == 0:
            #print "starting yapgvb process"
            try:
                print '--------------------------------------------'
                ygraph = yapgvb.Digraph('ROSGraph');
                gnodes = {}
                for anode in nodes:
                    gnodes[anode] = ygraph.add_node(label = anode)
                #print "added allnodes"
                #print flows
                for aflow in flows:
                    aflow_split= str(aflow[1]).split(':')
                    destination = aflow_split[0]
                    aflow_split_source = str(aflow[0]).split(':')
                    source = aflow_split_source[0]
                    gnodes[source] >> gnodes[destination]
                #print "done setting up graph for rendering"
                ygraph.layout(yapgvb.engines.dot)
                ygraph.render(output_file)
            finally:
                mpid = os.getpid()
                os.kill(mpid,9)
        else:
            os.wait()
            
        # Wrap up the image and send it out over a flow
        imToSend = Image.open(output_file)

        future_packet = pyImageString()
        future_packet.imageString = imToSend.tostring()

        future_packet.imageFormat = "RGB"
        future_packet.width, future_packet.height = imToSend.size

        out_FlowImageString.publish(future_packet)

        # don't loop too fast
        time.sleep(sleep_time)


        
if __name__ == '__main__':
    try:
        renderGraphMain(sys.argv, sys.stdout, os.environ)
    except Exception, e:
        traceback.print_exc()
        #attempt to log, may fail if logger is not properly initialized
        logger = logging.getLogger(NAME)
        logger.error(str(e)+"\n"+traceback.format_exc())        
        print "Exception is causing %s exit, check log for details"%NAME

    print "exiting"

 
