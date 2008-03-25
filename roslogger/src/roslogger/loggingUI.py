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
loggingUI

To run, invoke nodes/loggingUI
"""

import os, sys, getopt, traceback, logging
import rospy
import Tkinter
import time

NAME = 'loggingUI'
flow_widget_list = []
master = []
def usage(stdout, progname):
    print >>stdout, """%s [-h]

"""%progname+rospy.USAGE_ENV

class RecordManagement:
    global master

    def __init__(self, parent):
        self.parent = parent
        self.cb = []
        self.cbval = []
        self.draw_window = []

    def open_graph_display(self):
        if self.draw_window == []:
            #self.top = Tkinter.Toplevel(self.parent)
            #Tkinter.Label(self.top, text="Enter a Log Name:").pack()
            
            #self.log_name = Tkinter.Entry(self.top)
            #self.log_name.pack(padx=5)

            #b = Tkinter.Button(self.top, text="OK", command=self.button_cb)
            #b.pack(pady=5)
            self.load_graph()
            self.display_graph()
        else:
            print "Sorry window already open.  Please close record window before trying to open a new one."
    def start_logging_cb(self):
        master.addMachine('default', rospy.getRosRoot(), 'localhost', 22, '','')
        #print self.nodes
        #print self.flows
        for i in range(0, len(self.cb)):
            #            print self.cbval[i].get()
            if self.cbval[i].get() == 1:
                master.setParam('logging_node%s.name'%i,'hello_world')
                master.addNode('', 'logging_node%s'%i,'roslogger', 'logging_node','default',0)
                master.connectFlow(self.flows[i][0], 'logging_node%s.input'%i, 1)
        
        print "This will eventually start up nodes as specified and all"

    def stop_logging_cb(self):
        for i in range(0, len(self.cb)):
            #            print self.cbval[i].get()
            if self.cbval[i].get() == 1:
                master.killFlow(self.flows[i][0], 'logging_node%s.input'%i, 1)
                master.killNode('', 'logging_node%s'%i,'roslogger', 'logging_node','default',0)
        print "This will eventually shutdown nodes as specified and all"

    def button_cb(self):
        self.log_name_string = self.log_name.get()
        self.load_graph()
        self.display_graph()
        self.top.destroy()

    def save_logs(self):
        print "this will save logs to a server url from all the nodes"

    def close_graph_display(self):
        self.draw_window.destroy()
        self.draw_window = []
        self.cb = []
        self.cbval = []

    def reload_graph(self):
        self.load_graph()
        self.display_graph()

    def load_graph(self):
        print "Loading the graph from the master"
        status_code, statusMessage, [self.nodes,self.flows] = master.getGraph()
        #for flow in flows:

    def display_graph(self):
        if self.draw_window == []:  ### THIS DOESN'T WORK IF YOU CLOSE THE WINDOW WITH THE X
            print "creating a new window"
            self.draw_window = Tkinter.Toplevel(self.parent)
            self.draw_window.protocol("WM_DELETE_WINDOW", self.close_graph_display)
            
        for i in range(0,len(self.flows)):
            #print self.flows[i]
            Tkinter.Label(self.draw_window, text="Source:").grid(row=i, sticky=Tkinter.W)
            Tkinter.Label(self.draw_window, text=self.flows[i][0]).grid(row=i, column=1, sticky=Tkinter.W)
            Tkinter.Label(self.draw_window, text="Sink:").grid(row=i, column=2,sticky=Tkinter.W)
            Tkinter.Label(self.draw_window, text=self.flows[i][1]).grid(row=i, column=3, sticky=Tkinter.W)
            self.cbval.append(Tkinter.IntVar())
            self.cb.append( Tkinter.Checkbutton(self.draw_window, text="Log Me", variable=self.cbval[i]))
            self.cb[i].grid(row=i, column = 4, sticky = Tkinter.W)

        last_row = len(self.flows) + 1
        Tkinter.Label(self.draw_window, text="Please select what you \nwould like to record from above:").grid(row = last_row, column = 1)
        Tkinter.Button(self.draw_window, text="Click to reload", command=self.reload_graph).grid(row=last_row, column = 0)
        Tkinter.Button(self.draw_window, text="Save Logs", command=self.save_logs).grid(row=last_row+1, column = 3)
        Tkinter.Button(self.draw_window, text="Close window", command=self.close_graph_display).grid(row=last_row+1, column = 4)
        Tkinter.Label(self.draw_window, text="Log Name:").grid(row = last_row+1, column = 0)
        Tkinter.Label(self.draw_window, text="Folder URL:").grid(row = last_row+2, column = 0)
        self.log_name = Tkinter.Entry(self.draw_window)
        self.log_name.grid(row = last_row+1, column = 1)
        self.folder_url = Tkinter.Entry(self.draw_window)
        self.folder_url.grid(row = last_row+2, column = 1)
        #        Tkinter.Label(self.draw_window, text=self.log_name_string).grid(row = last_row+1, column = 1)
        
        Tkinter.Button(self.draw_window, text="Start Recording", command=self.start_logging_cb).grid(row=last_row, column = 3)
        Tkinter.Button(self.draw_window, text="Stop Recording", command=self.stop_logging_cb).grid(row=last_row, column = 4)
        self.draw_window.update()
        print "done displaying graph"    

class InitialPrompt:
    def __init__(self, parent):
        self.root = parent
        #top = self.top = Tkinter.Toplevel(parent)
        b = Tkinter.Button(self.root, text="Playback Mode", command = self.startPlaybackMode)
        b.pack(pady=5)
        b = Tkinter.Button(self.root, text="Record Mode", command = self.startRecordMode)
        b.pack(pady=5)
        self.d = RecordManagement(self.root)

    def startRecordMode(self):
        self.d.open_graph_display()

    def startPlaybackMode(self):
        print "sorry no playback management yet"

def loggingUIMain(argv, stdout, env):
    global master
    #check arguments for a help flag
    optlist, args = getopt.getopt(argv[1:], "h?", ["help"])
    if filter(lambda x: x in ["-h","-?","--help"], optlist):
        usage(stdout, argv[0])
        return

    # HACK to get things running
    serverUri = 'http://localhost:%s/'%rospy.DEFAULT_TEST_PORT
    print "TEST MODE: Providing ROS parameters manually"
    print "NOTE: assuming server at %s"%serverUri
    os.environ[rospy.ROS_PORT] = str(0) #any
    os.environ[rospy.ROS_MASTER_URI] = serverUri
    os.environ[rospy.ROS_NODE] = NAME

    
    

    rospy.start() #blocks until connected to master
    master = rospy.getMaster()
    print "ready to run"
    
    root = Tkinter.Tk()
    #Tkinter.Button(root,text="Load Graph").pack()
    #root.update()


    f = InitialPrompt(root)
    
    #   root.wait_window(f.top)
    #   print "got my initial prompt"
    #d = RecordManagement(root)
    #    print "record management has returned"
    #    root.wait_window(d.top)
    #    Tkinter.Button(root, text="test").pack()
    
    #    root.update()
    root.mainloop()

if __name__ == '__main__':
    try:
        loggingUIMain(sys.argv, sys.stdout, os.environ)
    except Exception, e:
        traceback.print_exc()
        #attempt to log, may fail if logger is not properly initialized
        logger = logging.getLogger(NAME)
        logger.error(str(e)+"\n"+traceback.format_exc())        
        print "Exception is causing %s exit, check log for details"%NAME

    print "exiting"

        
