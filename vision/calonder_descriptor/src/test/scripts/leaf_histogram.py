#!/usr/bin/python
from pylab import *
import sys

rc('text', usetex=True)

# load the data
bin_counts = load(sys.argv[1])
normed_counts = bin_counts[:,1] / sum(bin_counts[:,1])

bar(bin_counts[:,0], normed_counts, width=1)
xticks(bin_counts[:,0], ['$10^{%d}$' % -x for x in range(len(bin_counts[:,0]))])
yticks(arange(0.0, 1.1, 0.1))
title('Distribution from %s' % sys.argv[1])
xlabel('Bins')
ylabel('Fraction in bin')
show()
