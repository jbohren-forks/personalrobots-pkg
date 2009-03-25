#!/usr/bin/python

import sys, xml.sax, tempfile, subprocess

class Handler(xml.sax.ContentHandler):

    def __init__(self):
        self.active = False
        self.data = []
        self.actual_time = []
        self.cumul = []
        self.total = 0
        self.episode = 0
        tmp = tempfile.mkstemp()
#        self.datafile = tmp[0]
        self.datafile = open(tmp[1], 'w')
        self.datapath = tmp[1]
        
    def characters(self, data):
        if self.active:
            self.data.append(data)

    def endElement(self, tag):
        if not self.active:
            return
        if "actual_time_wall" == tag:
            actual_time = float(''.join(self.data))
            self.actual_time.append(actual_time)
            self.total += actual_time
            self.cumul.append(self.total)
        elif "episode_id" == tag:
            episode = int(''.join(self.data))
            if episode != self.episode:
                self.dumpEpisode()
                self.actual_time = []
                self.cumul = []
                self.total = 0
                self.episode = episode
        self.active = False
        self.data = []
    
    def startElement(self, tag, attrs):
        if "actual_time_wall" == tag:
            self.active = True
        elif "episode_id" == tag:
            self.active = True
            
    def endDocument(self):
        gnuplot = subprocess.Popen("/usr/bin/gnuplot", bufsize=1, stdin=subprocess.PIPE)
        print >>gnuplot.stdin, 'set terminal png large'
        print >>gnuplot.stdin, 'set output "abs_actual.png"'
        print >>gnuplot.stdin, 'set xlabel "absolute planning time per iteration [s]"'
        print >>gnuplot.stdin, 'plot "%s" using ($1) title "abs time" with lines' % (self.datapath)
        print >>gnuplot.stdin, 'set output "abs_cumul.png"'
        print >>gnuplot.stdin, 'set xlabel "absolute cumulated planning time [s]"'
        print >>gnuplot.stdin, 'plot "%s" using ($2) title "abs cumul time" with lines' % (self.datapath)
        print >>gnuplot.stdin, 'set output "rel_actual.png"'
        print >>gnuplot.stdin, 'set xlabel "relative planning time per iteration [percent of final]"'
        print >>gnuplot.stdin, 'plot "%s" using ($3) title "rel time" with lines' % (self.datapath)
        print >>gnuplot.stdin, 'set output "rel_cumul.png"'
        print >>gnuplot.stdin, 'set xlabel "relative cumulated planning time [percent of final]"'
        print >>gnuplot.stdin, 'plot "%s" using ($4) title "rel cumul time" with lines' % (self.datapath)
        
    def dumpEpisode(self):
        print >>self.datafile, '\n\n#episode %d' % self.episode
        print >>self.datafile, '# abs_actual\tabs_cumul\trel_actual\trel_cumul'
        for ii in xrange(len(self.actual_time)):
            print >>self.datafile, '%f\t%f\t%f\t%f' % (self.actual_time[ii],
                                                       self.cumul[ii],
                                                       self.actual_time[ii] / self.total,
                                                       self.cumul[ii] / self.total)

for arg in sys.argv[1:]:
    print 'parsing', arg
    xml.sax.parse(arg, Handler())
