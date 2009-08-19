#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#Author: Ian Goodfellow
PKG = 'ia3n_person_detection'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import roslib.rostime
import bisect
import logging
import chop_bag

def usage():
	print "./plot_bags.py <list of input bags> -o <output file (octave script)>"

def get_ranges_by_topic(inbags):
	gaps = chop_bag.find_gaps(inbags)
	for topic in gaps:
		gaps[topic] = chop_bag.gaps_to_chunks(gaps[topic])
	return gaps

def latex_escaped_str(str):
	pieces = str.split("_")
	rval = pieces[0]
	for piece in pieces[1:len(pieces)]:
		rval += ("\_"+piece)
	return rval

def plot_ranges(ranges, output):
	of = open(output, "w")

	ycoord = 1

	first = True

	for topic in ranges:
		for r in ranges[topic]:
			if first:
				x = r[0]
			of.write("line(["+str(r[0])+","+str(r[1])+"],["+str(ycoord)+","+str(ycoord)+"]);\n")	
		leftmost = ranges[topic][0][0]
		rightmost = ranges[topic][len(ranges[topic])-1][1]
		leftmost = roslib.rostime.Time.to_seconds(leftmost)
		rightmost = roslib.rostime.Time.to_seconds(rightmost)
		midpoint = 0.5*leftmost + 0.5*rightmost
		midpoint = roslib.rostime.Time.from_seconds(midpoint)
		of.write("text("+str(midpoint)+","+str(ycoord+0.5)+", \""+latex_escaped_str(topic)+"\");\n")
		ycoord += 1

	#draw 2 points to expand the window vertically and make everything visible
	of.write("line(["+str(x)+","+str(x)+"],[0,0]);\n")
	of.write("line(["+str(x)+","+str(x)+"],["+str(ycoord)+","+str(ycoord)+"]);\n")

	of.close()

def plot_bags(inbags,output):
	ranges = get_ranges_by_topic(inbags)
	plot_ranges(ranges, output)

if __name__ == '__main__':
	roslib.roslogging.configure_logging('plot_bags', logging.DEBUG, additional=['rospy', 'roslib'])


	import sys
	i = 1

	inbags = []
	outputSpecified = False	

	verbose = False

	#parse command line
	while i < len(sys.argv):
		if sys.argv[i] == "-o":
			i += 1
			if i >= len(sys.argv):
				rospy.logerror("-o flag must be followed by an argument, not end of input")
				usage()
				raise MyException("-o flag must be followed by an argument, not end of input")
			output = sys.argv[i]
			outputSpecified = True
		elif sys.argv[i] == "-v":
			verbose = True
		else:
			inbags.append(sys.argv[i])

		i += 1

	#verify parse
	if len(inbags) == 0:
		rospy.logerror("No input bags specified.")
		usage()
		raise MyException("")

	if not outputSpecified:
		rospy.logerrorr("No output file specified")
		usage()
		raise MyException("")

	plot_bags(inbags,output)
