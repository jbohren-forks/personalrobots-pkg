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
imageViewer

To run, invoke nodes/imageViewer
"""

import os, sys, getopt, traceback, logging
import rospy
import Tkinter
import Image, ImageTk
from pyImageViewer.pyImageString import *

NAME = 'imageViewer'

def usage(stdout, progname):
    print >>stdout, """%s [-h]

"""%progname+rospy.USAGE_ENV

class ImageViewer:
    def __init__(self):

        #initialize flows
        self.in_imageIn = InflowpyImageString("imageIn")
        #connect this node to the graph
        rospy.ready()

        # initialize Tkinter
        self.root = Tkinter.Tk()

        # setup the prompts at the top
        Tkinter.Label(text="Width:", anchor=Tkinter.E).grid(row=0,column=1)
        Tkinter.Label(text="Height:", anchor=Tkinter.E).grid(row=0,column=3)
        self.ewidth = Tkinter.StringVar()
        self.eheight = Tkinter.StringVar()
        self.ew = Tkinter.Entry(self.root, textvariable=self.ewidth)
        self.eh = Tkinter.Entry(self.root, textvariable=self.eheight)
        self.ew.grid(row=0, column= 2)
        self.eh.grid(row=0, column= 4)

        # A button to force resizing
        self.mb = Tkinter.Button(self.root, text="Click to resize", command=self.startup)
        self.mb.grid(row=0, column= 5)

        # A checkbox to turn on automatic resizing
        self.cbval = Tkinter.IntVar()
        self.mb2 = Tkinter.Checkbutton(self.root, text="Click to Autosize", variable=self.cbval, onvalue="1", offvalue="0")
        self.mb2.grid(row=0, column= 0)


        # Set the default window size
        self.ewidth.set(1000)
        self.eheight.set(1000)
        self.width = 1000 # These don't need to be preset, they are filled in startup before use
        self.height = 1000 # but I kept them here for clarity
        
        self.mimage = 0;  # This variable nees to be present for the canvas item to fill
        
        # space required for the header
        self.header_size = 40

        #initialize the first canvas
        self.mcanvas = Tkinter.Canvas(self.root)

        #run the startup/resize script
        self.startup()
        
        # start the loop to recieve flows
        self.imageViewerLoop()

        # start the Tkinter event loop
        Tkinter.mainloop()


    def startup(self):
        # Get the desired size from the Entry fields
        self.width = int(float(self.ewidth.get()))
        self.height = int(float(self.eheight.get()))

        # Resize the display if the desired size differs, and repack
        #        print "%dx%d"%(self.width,self.height)
        self.root.geometry(newGeometry="%dx%d"%(self.width,self.height+self.header_size))
        self.mcanvas.destroy()
        self.mcanvas = Tkinter.Canvas(self.root,  width =self.width ,height=self.height)
        self.mcanvas.grid(row=1, column= 0, columnspan=6, sticky=Tkinter.NW)
        
    def imageViewerLoop(self):

            #read/publish flows
            # - read inflow data
            imageInData = self.in_imageIn.receive()
            # get the image from the flow
            self.image = Image.fromstring(imageInData.imageFormat, (imageInData.width, imageInData.height), imageInData.imageString)

            # if auto resizing resize window to fit around image
            if self.cbval.get()==1:
                newgeometry_string = "%dx%d"%(self.image.size[0],self.image.size[1]+self.header_size)
                if self.root.winfo_geometry().split('+')[0] != newgeometry_string:
                    self.ewidth.set(self.image.size[0])
                    self.eheight.set(self.image.size[1])
                    self.startup()
                                
            # if the image is bigger than the window resize image to fit on screen
            if self.image.size[0] > self.width:
                newsize = (self.width, float(self.width)/self.image.size[0]*self.image.size[1])
                self.image = self.image.resize(newsize)
            if self.image.size[1] > self.height:
                newsize = (float(self.height)/self.image.size[1]*self.image.size[0],self.height)
                self.image = self.image.resize(newsize)

            # convert the Image to a ImageTk
            self.tkimage = ImageTk.PhotoImage(self.image)
            # place the image into the canvas
            self.mimage = self.mcanvas.create_image(0,0, anchor=Tkinter.NW, image=self.tkimage)

            self.root.after(1, self.imageViewerLoop) #looping method that allows Tkinter events to work correctly

def imageViewerMain(argv, stdout, env):
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
    os.environ[rospy.ROS_NODE] = NAME
    os.environ[rospy.ROS_PORT] = str(0) # any

    # Create an instance of the ImageViewer Class which actually has everything inside it.  
    mc = ImageViewer()



        
if __name__ == '__main__':
    try:
        imageViewerMain(sys.argv, sys.stdout, os.environ)
    except Exception, e:
        traceback.print_exc()
        #attempt to log, may fail if logger is not properly initialized
        logger = logging.getLogger(NAME)
        logger.error(str(e)+"\n"+traceback.format_exc())        
        print "Exception is causing %s exit, check log for details"%NAME

    print "exiting"

        
