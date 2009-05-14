#!/usr/bin/env python

"""
bin2c.py: Convert binary data to C array

Usage: bin2c.py <input file>
"""

import sys
import os.path

OPL = 12 # Octets per line

def Usage(msg = None):
 if msg:
   print msg
 print "Usage: bin2c.py <input file>"
 sys.exit(-1)

def main(argv):
 if len(argv) != 2:
   Usage()

 filename = argv[1]
 base = os.path.basename(filename)
 name = os.path.splitext(base)[0]

 data = open(argv[1]).read()
 octets = ["%#02x" % ord(x) for x in data]

 print "uint8_t %s[] = {" % name
 for i in xrange(0, len(octets), OPL):
   print "  " + ", ".join(octets[i:i+OPL]) + ","
 print "};"

if __name__ == "__main__":
 main(sys.argv)
