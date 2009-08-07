#!/usr/bin/python

import os
import sys
import string

def workFile(filename):
    f = open(filename, "r")
    fstr = f.read()
    f.close()
        
    if string.find(fstr, search_str) != -1:
        #print "Found match in", filename
        fstr = string.replace(fstr, search_str, replace_str)
        f = open(filename, "w")
        f.write(fstr)
        f.close
    #else:
        #print "No match in", filename

def workDir(filename):
    if os.path.isfile(filename):
        workFile(filename)
    elif os.path.isdir(filename):
        for filename2 in os.listdir(filename):
            workDir(os.path.join(filename, filename2))

def main(argv):
    print "Replacing", argv[0], "with", argv[1]

    search_file = open(argv[0], "r")
    global search_str
    search_str = search_file.read()

    replace_file = open(argv[1], "r")
    global replace_str
    replace_str = replace_file.read()

    workDir(argv[2])

    
if __name__ == "__main__":
    if(len(sys.argv) < 4):
        print "usage: ", sys.argv[0], " searchfile replacefile dir"
    else:
        main(sys.argv[1:])


