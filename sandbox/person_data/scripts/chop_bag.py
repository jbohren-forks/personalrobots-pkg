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

#Author: Ian Goodfellow (ia3n@cs.stanford.edu)

PKG = 'person_data'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import roslib.rostime
import bisect
import logging
import os

class MyException:
	def __init__(self, msg):
		self.msg = msg

#returns true iff two ranges intersect
def intersect(tuple1,tuple2):
	return tuple2[0] <= tuple1[1] and  tuple2[1] >= tuple1[0]

#conversts a list of gaps to a list of times that are not gaps
def gaps_to_chunks(gaps):
	#there are always 2 gaps: from the start of time to first message, from last message to end of time
	assert len(gaps) >= 2

	chunks = []

	idx = 1
	while idx < len(gaps):
		chunks.append((gaps[idx-1][1],gaps[idx][0]))
		idx+= 1

	return chunks
	

def chop_bags(inbags,outdir):
	write_bags_in_ranges(inbags,outdir,gaps_to_chunks(merge_gaps(find_gaps(inbags))))

def write_bags_in_ranges(inbags,outdir,ranges):
	if not os.path.exists(outdir):
		os.mkdir(outdir)
	else:
		if not os.path.isdir(outdir):
			raise MyException("A file blocks the creations of directory "+outdir)

	if len(ranges) == 0:
		return

	dirs = []
	idx = 0
	for r in ranges:
		dirs.append(outdir+"/scene_"+str(idx))

		if not os.path.exists(dirs[idx]):
			os.mkdir(dirs[idx])
		else:
			if not os.path.isdir(dirs[idx]):
				raise MyException("A file blocks the creations of directory "+outdir)

		idx += 1

	for inbag in inbags:
		i = 0
		r = ranges[i]

		#if true, reading messages and discarding them until we reach r
		#otherwise, emitting messages into rebag
		readingToRange = True

		for (topic,msg,t) in rosrecord.logplayer(inbag, raw=True):
			if readingToRange:
				if t >= r[0]:
					#open the bag
					bagfile = dirs[i]+"/"+inbag.split("/").pop()
					rospy.loginfo("Writing "+bagfile)
					rebag = rosrecord.Rebagger(bagfile)
					readingToRange = False

			#note that I don't use else here-- that way we can quite readingToRange on a message and then emit it
			if not readingToRange:
				if t <= r[1]:
					rebag.add(topic,msg,t,raw=True)
				else:
					i = i+1
					if i == len(ranges):
						break
					r = ranges[i]
					readingToRange = True



def find_gaps(inbags):
	deltas={}
	deltas["/wide_stereo/raw_stereo_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/narrow_stereo/raw_stereo_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/tilt_scan_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/base_scan_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/mechanism_state_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/laser_tilt_controller/laser_scanner_signal"]=roslib.rostime.Duration.from_seconds(2.0)
	deltas["/odom_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	deltas["/tf_message_muxed"]=roslib.rostime.Duration.from_seconds(0.5)
	long_duration = roslib.rostime.Duration.from_seconds(5)

	epsilon=roslib.rostime.Duration.from_seconds(0.001)
	beginningOfTime=roslib.rostime.Time.from_seconds(0)

	gaps = {}

	#assumes that no topic appears in more than one inbag
	prevtime = {}
	for inbag in inbags:
		
		for i,(topic,msg,t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
			#print (i,topic,t)
			#print roslib.rostime.Duration.from_seconds(0.5)


			if not topic in deltas:
				rospy.loginfo("No delta defined for topic "+topic)
				raise MyException("Grr")

			delta = deltas[topic]

			#if this topic has appeared before
			if topic in prevtime:
				#print "\t"+str(t-prevtime[topic])
			
				#check
				if t-prevtime[topic] > delta:
					rospy.loginfo(topic+": Found gap of "+str(roslib.rostime.Duration.to_seconds(t-prevtime[topic]))+" seconds")
					gaps[topic].append((prevtime[topic]+epsilon, t-epsilon))
			else: #this is the first instance of this topic, everything up until now is a gap
				gaps[topic] = []
				gaps[topic].append((beginningOfTime,t-epsilon))

			#Record this time for the next iteration
			prevtime[topic]=t

	#Find the end of time
	endOfTime = beginningOfTime
	for topic in prevtime:
		rospy.loginfo(topic+" ended at "+str(roslib.rostime.Time.to_seconds(prevtime[topic])))
		if prevtime[topic] > endOfTime:
			endOfTime = prevtime[topic]
		

	#All time after the ending is a gap
	for topic in prevtime:
		assert prevtime[topic] <= endOfTime
		if prevtime[topic] < endOfTime - long_duration:
			rospy.logerr(topic+" ended "+str(roslib.rostime.Duration.to_seconds(endOfTime-prevtime[topic]))+" seconds before the final topic")
		gaps[topic].append((prevtime[topic],endOfTime+epsilon))
			
	#Find the beginning of time
	beginningOfTime = endOfTime
	for topic in gaps:
		(ignore, first) = gaps[topic][0]
		if first < beginningOfTime:
			beginningOfTime = first

	#Back-patch the gaps to use this tighter beginningOfTime rather than 0
	for topic in gaps:
		(ignore, first) = gaps[topic][0]
		assert first >= beginningOfTime
		gaps[topic][0] = (beginningOfTime - epsilon, first)
			
		if first - long_duration > beginningOfTime:
			rospy.logerr(topic+" began "+str(roslib.rostime.Duration.to_seconds(first-beginningOfTime))+" seconds after the first topic")

	return gaps


def merge_gaps(gaps):
	merged_gaps = []

	for topic in gaps:
		for gap in gaps[topic]:
			#search out all intersecting merged_gaps
			intersecters = []

			idx = bisect.bisect_left(merged_gaps, gap)


			ridx = idx
			while ridx < len(merged_gaps):
				if intersect(gap,merged_gaps[ridx]):
					intersecters.append(ridx)
					ridx += 1
				else:
					break
		
			lidx = idx - 1
			while lidx >= 0:
				if intersect(gap,merged_gaps[lidx]):
					intersecters.append(lidx)
					lidx -= 1
				else:
					break			


			#Grow this gap so it includes all intersecting gaps
			#Delete all intersecting gaps
			intersecters.sort()
			intersecters.reverse()
			for intersecter in intersecters:
				start = min(merged_gaps[intersecter][0], gap[0])
				end = max(merged_gaps[intersecter][1], gap[1])
				gap = (start,end)
				del merged_gaps[intersecter]
				idx = intersecter

			#insert this gap into the list		
			merged_gaps.insert(idx,gap)	

	for i in range(1,len(merged_gaps)):
		curr = merged_gaps[i]
		prev = merged_gaps[i-1]
		assert curr[0] < curr[1]
		assert curr[0] > prev[1]
		rospy.loginfo("merged gap: "+str(curr))

	return merged_gaps

	

def usage():
	print "./chop_bags.py <list of input bags> -o <output directory>"

if __name__ == '__main__':
	roslib.roslogging.configure_logging('chop_bag', logging.DEBUG, additional=['rospy', 'roslib'])


	import sys
	i = 1

	inbags = []
	outdirSpecified = False	

	verbose = False

	#parse command line
	while i < len(sys.argv):
		if sys.argv[i] == "-o":
			i += 1
			if i >= len(sys.argv):
				rospy.logerror("-o flag must be followed by an argument, not end of input")
				usage()
				raise MyException("-o flag must be followed by an argument, not end of input")
			outdir = sys.argv[i]
			outdirSpecified = True
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

	if not outdirSpecified:
		rospy.logerrorr("No output directory specified")
		usage()
		raise MyException("")

	chop_bags(inbags,outdir)
	
