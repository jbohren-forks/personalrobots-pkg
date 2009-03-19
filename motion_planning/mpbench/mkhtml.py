#!/usr/bin/python

# there's got to be a better way...
import commands, subprocess
foo = commands.getstatusoutput("rospack find mpbench")
if 0 != foo[0]:
    print "could not rospack find mpbench"
    sys.exit(1)
mpbench = "%s/mpbenchmark" % foo[1]

worldspec = []
worldspec.append("pgm::../data/willow-clip0-r50.pgm:../data/willow-clip0-r50-a.xml")

plannerspec = []
plannerspec.append("ad:3dkin")

robotspec = []
for rot_speed in [60, 600, 6000]:
    robotspec.append("pr2:325:460:600:%d" % rot_speed)

costmapspec = []
costmapspec.append("sfl:50:325:460:550")
costmapspec.append("sfl:50:1:2:3")

out = open("index.html", 'w')
print >>out, '''<html>
<body>

<h1>Setup</h1>
<ul>
 <li> see how 3DKIN fares in snippet of WG map </li>
 <li> vary the "time to turn 45 deg" parameter </li>
 <li> change the costmap's expansion parameters, but not the robot footprint </li>
</ul>

<h1>AD planner</h1>
<table border="1" cellpadding="2">
 <tr><td>&nbsp;</td><th>slow rotation</th><th>nominal rotation</th><th>fast rotation</th></tr>'''
for cmidx in [0, 1]:
    if 0 == cmidx:
        print >>out, "  <tr><td>enlarged obstacles</td>"
    else:
        print >>out, "  <tr><td>raw obstacles</td>"
    for rs in robotspec:
        args = "-w %s -p %s -r %s -c %s" % (worldspec[0], plannerspec[0], rs, costmapspec[cmidx])
        foo = commands.getstatusoutput(mpbench + " " + args + " -X")
        if 0 != foo[0]:
            print "could not get basename"
            sys.exit(1)
        basename = foo[1]
        conslogname = "cons-%s.txt" % basename
        conslog = open(conslogname, 'w')
        print 'running with: ' + args
        retcode = subprocess.call([mpbench,
                                   "-w", worldspec[0],
                                   "-p", plannerspec[0],
                                   "-r", rs,
                                   "-c", costmapspec[cmidx],
                                   "-W"],
                                  stdout=conslog,
                                  stderr=subprocess.STDOUT)
        if 0 != retcode:
            print >>out, '  <td>failed with code %d, see <a href="%s">console output</a></td>' % (retcode, conslogname)
        else:
            print >>out, '  <td><table border="0" cellpadding="1">'
            print >>out, '   <tr><td colspan="3">./mpbenchmark ' + args + '</td></tr>'
            print >>out, '   <tr>'
            print >>out, '    <td><a href="' + basename + '.txt">log</a></td>'
            print >>out, '    <td><a href="' + conslogname + '">console</a></td>'
            print >>out, '    <td><a href="' + basename + '.result.xml">result</a></td>'
            print >>out, '   </tr><tr>'
            print >>out, '    <td colspan="3"><a href="' + basename + '.png">'
            print >>out, '     <img src="small-' + basename + '.png" alt="screenshot"></a></td>'
            print >>out, '   </tr><tr>'
            print >>out, '    <td colspan="3">'
            summary = open(basename + '.html')
            for line in summary:
                print >>out, '     ', line
            print >>out, '    </td>'
            print >>out, '   </tr></table></td>'
    print >>out, ' </tr>'
print >>out, '</table></body></html>'
out.close()
