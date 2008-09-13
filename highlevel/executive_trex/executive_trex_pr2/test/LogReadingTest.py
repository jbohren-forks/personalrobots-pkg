#!/usr/bin/env python


import time

class RealInterval:
    def __init__(self, variable, maxvalue, maxtolerance, minvalue=False, mintolerance=False):
        if (minvalue == False):
            minvalue = maxvalue
        if (mintolerance == False):
            mintolerance = maxtolerance
            
        self.variable = variable
        self.maxvalue = maxvalue
        self.maxtolerance = maxtolerance
        self.minvalue = minvalue
        self.mintolerance = mintolerance

    
    def inRange(self, min, max):
        dmin = float(min) - float(self.minvalue)
        dmax = float(max) - float(self.maxvalue)
        if (dmin < 0):
            dmin = -dmin
        if (dmax < 0):
            dmax = -dmax
        return (dmin < self.mintolerance and dmax < self.maxtolerance)

    def compare(self, line):
        values = line.split(" ")
        min = False
        for v in values:
            if (min != False):
                split = v.split("]")
                if (len(split) > 0):
                    return self.inRange(min, split[0].strip())
            else:
                splitEq = v.split("==")
                if (len(splitEq) > 1):
                    if (self.variable == splitEq[0]):
                        splitLeft = splitEq[1].split("[")
                        if (len(splitLeft) > 1):
                            if (splitLeft[0] == "REAL_INTERVAL:CLOSED"):
                                min = splitLeft[1].strip().strip(",")
        
        return False

class LogOnMessage:
    def __init__(self, timeline, token, arguments):
        self.timeline = timeline
        self.token = token
        self.arguments = arguments
    def compare(self, line):
        values = line.split(" ")
        if (values[0] != "ON"):
            return False
        if (values[2] != "ASSERT"):
            return False
        if (values[4] != "WITH"):
            return False
        if (len(values) < 4):
            return False

        for a in self.arguments:
            if (a.compare(line) == False):
                return False
        return True


class LogReadingTest:
    def __init__(self, logfile, messages):
        self.logfile = logfile
        self.line = 0
        self.message = 0
        self.messages = messages
        self.debugEnable = False
        self.stop = False
        
    def debug(self, a):
        if (self.debugEnable):
            print a

    def readLogFile(self):
        self.debug(self.logfile)
        try:
            file = open(self.logfile, "r")
        except IOError, e:
            print "Could not open file: " + self.logfile
            return
        lineNum = 0
        for line in file:
            lineNum = lineNum + 1
            if (lineNum > self.line):
                self.line = lineNum
                line = line.strip()
                self.debug("Added line: " + line)
                if (len(self.messages) > self.message):
                    if (self.messages[self.message].compare(line)):
                        self.debug("Found Match")
                        self.message = self.message + 1

    def getPassed(self):
        return (len(self.messages) <= self.message)

    def run(self):
        while 1:
            if (self.stop):
                return
            time.sleep(1.0)
            self.readLogFile()
            



if __name__ == '__main__':
    print "Running test test."
    lr = LogReadingTest("wpc.0.output/latest/TREX.log", [LogOnMessage("baseState", "BaseState.Holds", [RealInterval("x", 23, 1)])])
    lr.debugEnable = True
    lr.readLogFile()
    print lr.getPassed()

