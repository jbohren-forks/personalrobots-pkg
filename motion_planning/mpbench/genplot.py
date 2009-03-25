#!/usr/bin/python

import sys, xml.sax, subprocess, os, getopt

class Handler(xml.sax.ContentHandler):
    
    def reset(self):
        self.actual_time = []
        self.cumul_time = []
        self.total_time = 0
        self.plan_length = []
        self.plan_rotation = []
        self.solution_cost = []
        self.n_expands = []
        
    def __init__(self, basename):
        self.active = False
        self.data = []
        self.episode = 0
        self.basename = basename
        self.datapath = basename + '.data'
        self.datafile = open(self.datapath, 'w')
        self.reset()

    def characters(self, data):
        if self.active:
            self.data.append(data)

    def endElement(self, tag):
        if not self.active:
            return
        if "actual_time_wall" == tag:
            actual_time = float(''.join(self.data))
            self.actual_time.append(actual_time)
            self.total_time += actual_time
            self.cumul_time.append(self.total_time)
            #print "[%d] actual_time %f" % (self.episode, actual_time)
        elif "episode_id" == tag:
            episode = int(''.join(self.data))
            if episode != self.episode:
                self.dumpEpisode()
                self.episode = episode
                self.reset()
        elif "plan_length" == tag:
            plan_length = float(''.join(self.data))
            self.plan_length.append(plan_length)
            #print "[%d] plan_length %f" % (self.episode, plan_length)
        elif "plan_angle_change" == tag:
            plan_rotation = float(''.join(self.data))
            self.plan_rotation.append(plan_rotation)
            #print "[%d] plan_rotation %f" % (self.episode, plan_rotation)
        elif "solution_cost" == tag:
            solution_cost = float(''.join(self.data))
            self.solution_cost.append(solution_cost)
            #print "[%d] solution_cost %f" % (self.episode, solution_cost)
        elif "number_of_expands" == tag:
            n_expands = int(''.join(self.data))
            self.n_expands.append(n_expands)
            #print "[%d] n_expands %f" % (self.episode, n_expands)
        self.active = False
        self.data = []
    
    def startElement(self, tag, attrs):
        if "actual_time_wall" == tag:
            self.active = True
        elif "episode_id" == tag:
            self.active = True
        elif "plan_length" == tag:
            self.active = True
        elif "plan_angle_change" == tag:
            self.active = True
        elif "solution_cost" == tag:
            self.active = True
        elif "number_of_expands" == tag:
            self.active = True
    
    def endDocument(self):
        gnuplot = subprocess.Popen("/usr/bin/gnuplot", bufsize=1, stdin=subprocess.PIPE)
        print >>gnuplot.stdin, 'set terminal png large'
        
        print 'creating %s_abs_actual_time.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_abs_actual_time.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "absolute planning time per iteration"'
        print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
        print >>gnuplot.stdin, 'set ylabel "time [s]"'
        print >>gnuplot.stdin, 'plot "%s" using ($1) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_abs_cumul_time.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_abs_cumul_time.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "absolute cumulated planning time per iteration"'
        print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
        print >>gnuplot.stdin, 'set ylabel "cumul time [s]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_rel_actual_time.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_rel_actual_time.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "relative planning time per iteration"'
        print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
        print >>gnuplot.stdin, 'set ylabel "time [% of total]"'
        print >>gnuplot.stdin, 'plot "%s" using ($3) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_rel_cumul_time.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_rel_cumul_time.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "relative cumulated planning time per iteration"'
        print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
        print >>gnuplot.stdin, 'set ylabel "cumul time [% of total]"'
        print >>gnuplot.stdin, 'plot "%s" using ($4) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_abs_plan_length.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_abs_plan_length.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "absolute path length per cumulated planning time"'
        print >>gnuplot.stdin, 'set xlabel "cumul time [s]"'
        print >>gnuplot.stdin, 'set ylabel "length [m]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2):($5) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_abs_plan_rotation.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_abs_plan_rotation.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "absolute path rotation per cumulated planning time"'
        print >>gnuplot.stdin, 'set xlabel "cumul time [s]"'
        print >>gnuplot.stdin, 'set ylabel "rotation [rad]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2):($6) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_rel_plan_length.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_rel_plan_length.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "relative path length per cumulated planning time"'
        print >>gnuplot.stdin, 'set xlabel "cumul time [s]"'
        print >>gnuplot.stdin, 'set ylabel "length [% of final]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2):($7) notitle with linespoints' % (self.datapath)
        
        print 'creating %s_rel_plan_rotation.png' % self.basename
        print >>gnuplot.stdin, 'set output "%s_rel_plan_rotation.png"' % self.basename
        print >>gnuplot.stdin, 'set x2label "relative path rotation per cumulated planning time"'
        print >>gnuplot.stdin, 'set xlabel "cumul time [s]"'
        print >>gnuplot.stdin, 'set ylabel "rotation [% of final]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2):($8) notitle with linespoints' % (self.datapath)
        
        if len(self.solution_cost) > 0:
            print 'creating %s_sol_cost.png' % self.basename
            print >>gnuplot.stdin, 'set output "%s_sol_cost.png"' % self.basename
            print >>gnuplot.stdin, 'set x2label "solution cost per cumulated planning time"'
            print >>gnuplot.stdin, 'set xlabel "cumul time [s]"'
            print >>gnuplot.stdin, 'set ylabel "cost [#]"'
            print >>gnuplot.stdin, 'plot "%s" using ($2):($9) notitle with linespoints' % (self.datapath)
        
        if len(self.n_expands) > 0:
            print 'creating %s_n_expands.png' % self.basename
            print >>gnuplot.stdin, 'set output "%s_n_expands.png"' % self.basename
            print >>gnuplot.stdin, 'set x2label "number of state expansions per iteration"'
            print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
            print >>gnuplot.stdin, 'set ylabel "expands [#]"'
            print >>gnuplot.stdin, 'plot "%s" using ($10) notitle with linespoints' % (self.datapath)

            print 'creating %s_expand_speed.png' % self.basename
            print >>gnuplot.stdin, 'set output "%s_expand_speed.png"' % self.basename
            print >>gnuplot.stdin, 'set x2label "state expansion speed per iteration"'
            print >>gnuplot.stdin, 'set xlabel "iteration [#]"'
            print >>gnuplot.stdin, 'set ylabel "expands [1/s]"'
            print >>gnuplot.stdin, 'plot "%s" using ($0):($10)/($1) notitle with linespoints' % (self.datapath)
            
    def dumpEpisode(self):
        print >>self.datafile, '\n\n#episode %d' % self.episode
        print >>self.datafile, '# abs_actual_time\tabs_cumul_time\trel_actual_time\trel_cumul_time\tabs_plan_length\tabs_plan_rotation\trel_plan_length\trel_plan_rotation\tn_expands'
        for ii in xrange(len(self.actual_time)):
            line = '%f\t%f\t%f\t%f' % (self.actual_time[ii],
                                       self.cumul_time[ii],
                                       100 * self.actual_time[ii] / self.total_time,
                                       100 * self.cumul_time[ii] / self.total_time)
            line = line + '\t%f\t%f\t%f\t%f' % (self.plan_length[ii],
                                                self.plan_rotation[ii],
                                                100 * self.plan_length[ii] / self.plan_length[-1],
                                                100 * self.plan_rotation[ii] / self.plan_rotation[-1])
            if len(self.solution_cost) > 0:
                line = line + '\t%f' % self.solution_cost[ii]
            else:
                line = line + '\t0'
            if len(self.n_expands) > 0:
                line = line + '\t%d' % self.n_expands[ii]
            else:
                line = line + '\t0'
            print >>self.datafile, line

optlist, args = getopt.getopt(sys.argv[1:], 'i:o:')
infilename = ""
basename = ""
for opt in optlist:
    if opt[0] == "-i":
        infilename = opt[1]
    elif opt[0] == "-o":
        basename = opt[1]
if infilename == "":
    print "error: use -i option to specify an input XML result file"
    exit(42)
if basename == "":
    basename = "plot-" + infilename
xml.sax.parse(infilename, Handler(basename))
