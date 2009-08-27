#!/usr/bin/env python

__doc__ = """
execution_monitor
Execution Monitor for TREX over ROS."""

# ROS Stuff
PKG = 'trex_ros' # this package name
NAME = 'trex_ros'

import roslib; roslib.load_manifest(PKG)
import rospy
from trex_ros.msg import *

# System modules
import sys,os
import unittest
import signal
import threading, time
import gtk

# TREX modules
from TREX import *
from TREX.core import *
from TREX.widgets_gtk import *

# ROSDbListener
# This class allows the TREX execution monitor to function over ROS messages
# instead of a local filesystem. It caches the last N ticks, and provides
# semantics similar to the TREX DbReader python class.

class ROSDbListener():
  def __init__(self):
    # Store a dictionary where self.ticks[ticknum] = a dictionary of DbCore
    # structures, keyed on reactor name
    self.ticks = {}

    # Store the topic subscribers dict, which is keyed on reactor name
    self.plan_topics = {}

    # Store the next pending tick
    # This wll be a tick number referencing the latest entry in self.ticks
    # that has been populated by all of the subscribed topics
    self.latest_full_tick = None

    self.cache_size = 0

  # Set the tick cache size
  # This will cause the ROSDbListener to remember the last size states before
  # self.latest_full_tick
  def set_cache_size(self,size):
    self.cache_size = size

  # This is used to syncrhonize the execution monitor with
  # the currently running TREX reactors. It will periodically check that 
  # it is monitoring the full set of reactors
  def update_available_topics(self):
    # Get all topics broadcasting PlanDescriptions
    plan_topics = [topic[0] for topic in rospy.get_published_topics() if topic[1]=='trex_ros/PlanDescription']
    # Subscribe to them 
    for topic in plan_topics:
      if topic not in self.plan_topics.keys():
	self.plan_topics[topic] = rospy.Subscriber(topic, PlanDescription, self.update_reactor, topic)
    # Unsubscribe from topics not in the available topics but that we are currently listening to
    for topic in self.plan_topics.keys():
      if topic not in plan_topics:
    	del self.plan_topics[topic]

  # Subscriber callback when a plan gets published
  def update_reactor(self, plan_desc, topic_name):
    # Get tick from plan description
    tick = (plan_desc.tick,0)
    # Get reactor name from topic name
    reactor_name = topic_name#.split('/')[1]
    
    # Check if this tick has already been started by another reactor
    if not self.ticks.has_key(tick):
      # Create a dict for this tick
      self.ticks[tick] = dict()

    # Create DbCore for this reactor
    db_core = DbCore(tick,reactor_name)

    # Populate this DbCore 
    # Iterate over internal timelines
    for tl_desc in plan_desc.internal:
      timeline = Timeline(0,tl_desc.name,Timeline.INTERNAL)
      for token_desc in tl_desc.tokens:
	token = Token(token_desc.key,token_desc.name,0,0)
	token.start = token_desc.start
	token.end = token_desc.end
	timeline.tokens.append(token)
      db_core.int_timelines[tl_desc.name] = timeline
    
    # Iterate over external timelines
    for tl_desc in plan_desc.external:
      timeline = Timeline(0,tl_desc.name,Timeline.EXTERNAL)
      for token_desc in tl_desc.tokens:
	token = Token(token_desc.key,token_desc.name,0,0)
	token.start = token_desc.start
	token.end = token_desc.end
	timeline.tokens.append(token)
      db_core.ext_timelines[tl_desc.name] = timeline

    # Append the DbCore to the ticks dict
    self.ticks[tick][reactor_name] = db_core

    # Update the current tick if necessary
    self.update_latest_tick(tick)

  # Update the latest tick if it is newer than the current latet 
  # full tick and all reactors have been loaded
  def update_latest_tick(self,new_tick):
    # Update the current topics
    self.update_available_topics()

    # Return if there is a subscribed reactor that is not yet loaded into the proposed tick
    for reactor_name in self.plan_topics.keys():
      if reactor_name not in self.ticks[new_tick].keys():
	return

    # Remove old ticks
    for tick in self.ticks.keys():
      if tick[0] < new_tick[0] - 1:
	del self.ticks[tick]

    # If we get here, all reactors are loaded
    self.latest_full_tick = new_tick

  ###########################################################################
  # Methods to fake out DbReaderWindow

  def get_available_reactors(self,log_path):
    # Update the current topics
    self.update_available_topics()

    return self.plan_topics.keys()

  # Get a list of the available ticks
  def get_available_db_cores(self,log_path,reactor_name):
    if self.latest_full_tick:
      return [self.latest_full_tick]
    else:
      return []

  def get_available_assemblies(self,log_path,reactor_name):
    return []

  def get_available_conflicts(self,log_path,reactor_name):
    return []

  def load_db(self,log_path,reactor_name,tick):
    # Create a new Assembly for storing data
    db_core = self.ticks[tick][reactor_name]

    # Get a list of all the available conflicts
    db_core.conflicts = []

    # Return constructed db_core
    return db_core

  def load_assembly(self,log_path,reactor_name,tick):
    return Assembly()


def main():
  # Process cli arguments
  if len(sys.argv) > 1:
    log_path = sys.argv[1]
  else:
    log_path = "."

  # Initialize node
  rospy.init_node("trex_execution_monitor")
  #param_setter = threading.Thread(target=rospy.spin)
  #param_setter.start()

  # Window configuration for a window that should not be allowed to be closed
  WINDOW_FUNCTIONS = gtk.gdk.FUNC_MOVE | gtk.gdk.FUNC_RESIZE | gtk.gdk.FUNC_MINIMIZE | gtk.gdk.FUNC_MAXIMIZE;

  # Initialize gtk multithread support
  gtk.gdk.threads_init()

  # Create db reader window
  db_reader_window = DbReaderWindow(db_reader=ROSDbListener(),log_path=log_path)
  db_reader_window.w.connect("destroy",gtk.main_quit)
  # Disable log path buttons
  db_reader_window.path_chooser.set_sensitive(False)
  db_reader_window.reactor_chooser.set_sensitive(False)
  db_reader_window.load_available_assembly_check.set_active(False)
  db_reader_window.load_available_assembly_check.emit("toggled")
  db_reader_window.load_available_assembly_check.set_sensitive(False)
  db_reader_window.assembly_ticks_only_check.set_sensitive(False)
  
  # Create timelien viewer window
  timeline_window = TimelineWindow()
  timeline_window.w.window.set_functions(WINDOW_FUNCTIONS)
  db_reader_window.register_listener(timeline_window.set_db_cores)

  ############################################################

  # Create conflict list window
  def SetSourceFile(tick, reactor):
    db_reader_window.tick_entry.set_text(db_reader_window.format_tick(tick))
    db_reader_window.go_but.emit("clicked")
    
    timeline_window.set_visible_reactor(reactor)

  conflict_list_window = ConflictListWindow()
  conflict_list_window.register_activate_callback(SetSourceFile)
  db_reader_window.register_listener(conflict_list_window.set_db_cores)

  # Bring the db reader forward
  db_reader_window.w.present()

  # Set up signal handler
  signal.signal(signal.SIGINT, signal.SIG_DFL) 

  gtk.main()

if __name__ == '__main__':
  main()
