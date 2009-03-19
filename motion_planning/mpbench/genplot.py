#!/usr/bin/python

import sys, xml.sax

class Handler(xml.sax.ContentHandler):

    def __init__(self):
        self.active = False
        self.data = []
        self.actual_time_wall = []

    def characters(self, data):
        if self.active:
            self.data.append(data)

    def endElement(self, tag):
        if not self.active:
            return
        if "actual_time_wall" == tag:
            self.actual_time_wall.append(''.join(self.data))
        self.active = False
        self.data = []
    
    def startElement(self, tag, attrs):
        if "actual_time_wall" == tag:
            self.active = True

    def endDocument(self):
        for atw in self.actual_time_wall:
            print '  ', atw

for arg in sys.argv[1:]:
    print 'parsing', arg
    xml.sax.parse(arg, Handler())
