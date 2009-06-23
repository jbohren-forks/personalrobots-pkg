#!/usr/bin/python

# Header scanner...
import sys, os

my_dir = os.getcwd()
files = []
debug = False
directories = [my_dir]
help = False

for a in sys.argv[1:]:
    if (a == "--help" or a == "-help" or a == "-h"):
        help = True
    elif (a[0] == "-"):
        if(a[1] == "I"):
            dir = os.path.abspath(a[2:])
            if (dir not in directories):
                directories.append(dir)
        elif(a[1] == "B"):
            debug = True
            print "DEBUG ON, WD:", my_dir
        elif(a[1] == "S"):
             my_dir = a[2:]
        else:
            print "Reject argument:", a
            help = True
    elif(a.strip() != ""):
        if (os.path.isabs(a)):
            file = a
        else:
            file = os.path.join(my_dir, a)
        if (file not in files):
            files.append(file)
        

if (len(files) != 1 or help):
    if (not help):
        print "There are ", len(files), " files, there can only be one."
    print "Args:", sys.argv
    print "Help for", sys.argv[0]
    print "A script to scan nddl files for dependencies."
    print "  Usage:", sys.argv[0], " <file> <option 1> <option 2> <option 3> <option 4> .. <option N>"
    print "Where <file> is an nddl file."
    print "<option> is one of:"
    print "  -I<dir>: Include directory <dir>."
    print "  -B: Turn on debug prints and print working directory."
    print "  -S: Current source directory."
    sys.exit(1)


  
if (debug):
    print "Directories:"
    for dir in directories:
        print dir

size = 0
  
#Do this until all the files have been proscessed.
while (size != len(files)):
    if (debug):
        print "Size:", size, "Files:", len(files)
    size = len(files)
    newfiles = []
    for file in files:
        try:
            fp = open(file, "r")
            for linedirty in fp:
                line = linedirty.strip()
                if (line[0:len("#include")] == "#include"):
                    start = line.index("\"")
                    end = line.index("\"", start + 1)
                    newfiles.append(line[start + 1:end])
            fp.close()
        except:
            print "FAILURE: could not open", file
            sys.exit(2)
    
    for f in newfiles:
        indir = False
        for dir in directories:
            if (os.path.exists(os.path.join(dir, f)) and not indir):
                indir = dir
        if (not indir):
            print "FAILURE: could not find include file", f
            sys.exit(3)
        ff = os.path.join(indir, f)
        if (ff not in files):
            files.append(ff)
        

if (debug):
    print "Files:"

for file in files:
    print os.path.normpath(file)












