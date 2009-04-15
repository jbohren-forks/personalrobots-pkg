#!/usr/bin/python

# there's got to be a better way...
import commands, subprocess, sys
foo = commands.getstatusoutput("rospack find mpbench")
if 0 != foo[0]:
    print "could not rospack find mpbench"
    sys.exit(1)
mpbench = "%s/mpbenchmark" % foo[1]
genplot = "%s/genplot.py" % foo[1]

worldspec = []
worldexpl = []
worldspec.append("xml:../data/test-door-1.xml")
worldexpl.append("first door")
worldspec.append("xml:../data/test-door-2.xml")
worldexpl.append("second door")
worldspec.append("xml:../data/test-door-3.xml")
worldexpl.append("third door")
worldspec.append("xml:../data/test-door-4.xml")
worldexpl.append("fourth door")

plannerspec = []
plannerexpl = []
plannerspec.append("ad:xythetadoor:../data/pr2sides.mprim")
plannerexpl.append("AD door planner with sideways motion")
plannerspec.append("ad:xythetadoor:../data/pr2.mprim")
plannerexpl.append("AD door planner without sideways motion")

robotspec = []
robotexpl = []
for rot_speed in [600]:
    robotspec.append("pr2:325:460:600:%d" % rot_speed)
    robotexpl.append("rotation %g rad/s" % (1e-3 * rot_speed))

costmapspec = []
costmapexpl = []
costmapspec.append("sfl:25:325:460:550")
costmapexpl.append("fully inflated")

out = open("index.html", 'w')
print >>out, '<html><body>'

counter = 0
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
                basename = "mpbench%d" % counter
                counter += 1
                conslogname = "cons-%s.txt" % basename
                conslog = open(conslogname, 'w')
                args = '-w %s -p %s -r %s -c %s' % (worldspec[iworld],
                                                    plannerspec[iplanner],
                                                    robotspec[irobot],
                                                    costmapspec[icostmap])
                print 'basename:', basename, '\targs:', args
                retcode = subprocess.call([mpbench,
                                           "-w", worldspec[iworld],
                                           "-p", plannerspec[iplanner],
                                           "-r", robotspec[irobot],
                                           "-c", costmapspec[icostmap],
                                           "-x", basename,
                                           "-W"],
                                          stdout=conslog,
                                          stderr=subprocess.STDOUT)
                print >>out, '  <td><table border="0" cellpadding="1">'
                print >>out, '   <tr><td colspan="3">./mpbenchmark ' + args + '</td></tr>'
                if 0 != retcode:
                    print >>out, '   <tr><td>possibly broken: mpbenchmark <b>failed with code %d</b>, see <a href="%s">console output</a></td></tr>' % (retcode, conslogname)
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
                print >>out, '   </tr>'
                plotlogname = "plot-%s.txt" % basename
                plotlog = open(plotlogname, 'w')
                retcode = subprocess.call([genplot,
                                           "-i", basename + ".result.xml",
                                           "-o", basename],
                                          stdout=plotlog,
                                          stderr=subprocess.STDOUT)
                if 0 != retcode:
                    print >>out, '   <tr><td colspan="3">genplot failed with code %d, see <a href="%s">console output</a></td></tr>' % (retcode, plotlogname)
                else:
                    print >>out, '   <tr>'
                    print >>out, '    <td><a href="' + basename + '_plots.html">plots</a>'
                    print >>out, '    <td><a href="' + plotlogname + '">plot console</a>'
                    print >>out, '    <td><a href="' + basename + '.data">plot data</a>'
                    print >>out, '   </tr>'
                    plots = open(basename + '_plots.html', 'w')
                    print >>plots, '<html><body>'
                    print >>plots, '<h1>Summary Plots</h1>'
                    print >>plots, '<p>mpbench command line options: <pre>', args, '</pre></p>'
                    print >>plots, '<p>See also: <a href="' + plotlogname + '">plot console</a> and <a href="' + basename + '.data">plot data</a>.</p>'
                    for foo in 'abs_plan_length abs_plan_rotation rel_plan_length rel_plan_rotation abs_actual_time abs_cumul_time rel_actual_time rel_cumul_time sol_cost n_expands expand_speed'.split():
                        print >>plots, '<img src="' + basename + '_' + foo + '.png" alt="' + foo + '"><br>'
                    print >>plots, '</body></html>'
                    plots.close()
                print >>out, '</table></td>'
            print >>out, ' </tr>'
    print >>out, '</table>'
print >>out, '</body></html>'
out.close()
