#Author: Ian Goodfellow
#Pass it the ROBOT environment variable to see if it is OK
#TODO-- find out some way of bringing down the whole launch file

import sys

if len(sys.argv) != 2:
	for i in range(0,1000):
		print "Your ROBOT variable is not set"
	quit(-1)

if sys.argv[1] != "prf" and sys.argv[1] != "prg":
	for i in range(0,1000):
		print "Your ROBOT variable is not set"
	quit(-1)

quit(0)
