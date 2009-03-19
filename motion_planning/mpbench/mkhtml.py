#!/usr/bin/python

# there's got to be a better way...
import commands, subprocess, sys
foo = commands.getstatusoutput("rospack find mpbench")
if 0 != foo[0]:
    print "could not rospack find mpbench"
    sys.exit(1)
mpbench = "%s/mpbenchmark" % foo[1]

worldspec = []
worldexpl = []
worldspec.append("pgm:204:../data/willow-clip1-r25.pgm:../data/willow-clip1-r25-a.xml")
worldexpl.append("hallway to office")
worldspec.append("pgm:204:../data/willow-clip1-r25.pgm:../data/willow-clip1-r25-b.xml")
worldexpl.append("office to hallway")

plannerspec = []
plannerexpl = []
plannerspec.append("ad:xythetalat:../data/pr2.mprim")
plannerexpl.append("AD planner on lattice")
plannerspec.append("ara:xythetalat:../data/pr2.mprim")
plannerexpl.append("ARA planner on lattice")

robotspec = []
robotexpl = []
for rot_speed in [60, 600, 6000]:
    robotspec.append("pr2:325:460:600:%d" % rot_speed)
    robotexpl.append("rotation %g rad/s" % (1e-3 * rot_speed))

costmapspec = []
costmapexpl = []
costmapspec.append("sfl:25:325:460:550")
costmapexpl.append("fully inflated")
costmapspec.append("sfl:25:1:2:3")
costmapexpl.append("point obstacles")

out = open("index.html", 'w')
print >>out, '''<html><body>

<h1>Setup</h1>
<ul>
 <li> see how lattice planning fares in Sachin's office </li>
 <li> vary the "time to turn 45 deg" parameter </li>
 <li> change the costmap's expansion parameters, but not the robot footprint </li>
</ul>'''

for iplanner in xrange(len(plannerspec)):
    print >>out, '<h1>' + plannerexpl[iplanner] + '</h1>'
    print >>out, '<table border="1" cellpadding="2">'
    print >>out, ' <tr>'
    print >>out, '  <td>&nbsp;</td>'
    for rob in robotexpl:
        print >>out, '  <th>' + rob + '</th>'
    print >>out, ' </tr>'
    for iworld in xrange(len(worldspec)):
        for icostmap in xrange(len(costmapspec)):
            print >>out, ' <tr>'
            print >>out, '  <td>' + worldexpl[iworld] + '<br>' + costmapexpl[icostmap] + '</td>'
            for irobot in xrange(len(robotspec)):
                args = ' -w ' + worldspec[0] + \
                    ' -p ' + plannerspec[iplanner] + \
                    ' -r ' + robotspec[irobot] + \
                    ' -c ' + costmapspec[icostmap]
                foo = commands.getstatusoutput(mpbench + args + ' -X')
                if 0 != foo[0]:
                    print 'could not get basename'
                    sys.exit(1)
                basename = foo[1]
                conslogname = "cons-%s.txt" % basename
                conslog = open(conslogname, 'w')
                print 'running with:' + args
                retcode = subprocess.call([mpbench,
                                           "-w", worldspec[0],
                                           "-p", plannerspec[iplanner],
                                           "-r", robotspec[irobot],
                                           "-c", costmapspec[icostmap],
                                           "-W"],
                                          stdout=conslog,
                                          stderr=subprocess.STDOUT)
                if 0 != retcode:
                    print >>out, '  <td>failed with code %d, see <a href="%s">console output</a></td>' % (retcode, conslogname)
                else:
                    print >>out, '  <td><table border="0" cellpadding="1">'
                    print >>out, '   <tr><td colspan="3">./mpbenchmark' + args + '</td></tr>'
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
    print >>out, '</table>'
print >>out, '</body></html>'
out.close()
