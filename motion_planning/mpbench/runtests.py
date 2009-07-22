#!/usr/bin/python

# there's got to be a better way...
import commands, subprocess, sys
pkgdir = commands.getstatusoutput("rospack find mpbench")
if 0 != pkgdir[0]:
    print "could not rospack find mpbench"
    sys.exit(1)
mpbench = "%s/mpbenchmark" % pkgdir[1]

args = []
args.append("-p navfn")
args.append("-p estar")
args.append("-p ad:2d")
args.append("-p ad:2d16")
args.append("-p ad:3dkin")
args.append("-p ad:xythetalat:%s/data/pr2.mprim -c ros:25" % (pkgdir[1]))
args.append("-p ad:xythetadoor:%s/data/pr2.mprim -c ros:25 -w xml:%s/data/test-single-door.xml" % (pkgdir[1], pkgdir[1]))
args.append("-p ad:xythetalat:%s/data/pr2.mprim -c sfl:25" % (pkgdir[1]))
args.append("-p ad:xythetadoor:%s/data/pr2.mprim -c sfl:25 -w xml:%s/data/test-single-door.xml" % (pkgdir[1], pkgdir[1]))
args.append("-p ara:2d")
args.append("-p ara:2d16")
args.append("-p ara:3dkin")
args.append("-p ara:xythetalat:%s/data/pr2.mprim -c ros:25" % (pkgdir[1]))
args.append("-p ara:xythetadoor:%s/data/pr2.mprim -c ros:25 -w xml:%s/data/test-single-door.xml" % (pkgdir[1], pkgdir[1]))
args.append("-p ara:xythetalat:%s/data/pr2.mprim -c sfl:25" % (pkgdir[1]))
args.append("-p ara:xythetadoor:%s/data/pr2.mprim -c sfl:25 -w xml:%s/data/test-single-door.xml" % (pkgdir[1], pkgdir[1]))

failures = []
for iarg in xrange(len(args)):
    conslogname = "test%03d.txt" % iarg
    conslog = open(conslogname, 'w')
    callargs = [mpbench] + args[iarg].split() + ['-W']
    print callargs
    retcode = subprocess.call(callargs, stdout=conslog, stderr=subprocess.STDOUT)
    status = "%3d: %s (args: %s)" % (iarg, conslogname, args[iarg])
    if 0 != retcode:
        print "FAILED %s" % status
        failures.append("FAILED %s" % status)
    else:
        print "passed %s" % status

print "--------------------------------------------------"
if 0 == len(failures):
    print "all tests passed"
else:
    print "summary of failures:"
    for failure in failures:
        print failure

