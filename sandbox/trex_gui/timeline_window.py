#!/usr/bin/env python

# System modules
import sys,os
import random
import unittest
import threading, time

import colorsys
import math
import bisect

import cairo, pangocairo
import gtk, gtk.glade

# TREX modules
from db_core import DbCore,Timeline
from assembly import Assembly,Entity,Rule,Token,Slot,Variable

##############################################################################
# Timeline
#   This class represents a TREX timeline and is used for keeping track of
#   tokens and drawing state.
##############################################################################

class TimelineSprite():
  # Timeline constants
  INTERNAL, EXTERNAL = range(2)

  def __init__(self,timeline):
    # Store this object
    self.tl = timeline

    # Store the reactor that this timeline belongs to
    self.reactor_panel = None

    # Store a list of active tokens
    self.active_tokens = []
    self.new_active_tokens = []

    # Store a list of region boundaries for hit-testing
    self.hit_boundaries = []
    self.hit_tokens = []

    # Drawing variables
    self.earliest_tick = float("Inf")
    self.earliest_start = 0
    self.latest_end = 0
    self.latest_tick = float("Inf")
    self.row = 0

    # Add tokens
    self.add_tokens(timeline.tokens)

  # Add new tokens to this timeline
  def add_tokens(self,tokens):
    # Copy new active tokens into active tokens, reset new active tokens
    #self.active_tokens = self.active_tokens + self.new_active_tokens
    self.new_active_tokens = []
    # Copy only active tokens onto the token list
    for token in tokens:
      if token.slot_index == 0:
	self.new_active_tokens.append(token)

  # Get the color for this timeline
  def get_color(self):
    if self.reactor_panel:
      rgb = self.reactor_panel.color
    else:
      # No parent
      rgb = (0.3,0.3,0.3)

    return rgb

  # Get the token that was hit
  def get_hit_token(self,x):
    token_index = bisect.bisect_left(self.hit_boundaries, x)
    if token_index < len(self.hit_tokens):
      return (self.hit_tokens[token_index], self.hit_boundaries[token_index])
    else:
      return (None,0)

##############################################################################
# ReactorPanel
#   This is a GTK widget container class that draws all of the timelines for
#   a given reactor. It has two cairo drawing contexts which are separated by
#   a slider.
#   
#   ReactorPanel currently has two drawing modes for timelines:
#     Compact:	All tokens are compressed to the smallest area necessary to
#		print their names. This shows only temporal ordering.
#     Metric:	All tokens are expanded (or compressed) to maintiain relative
#		scale over time. This shows both temporal ordering and
#		temporal scale, which makes it harder to observe terse tokens
#		in the same context as long-running ones.
##############################################################################

class ReactorPanel():
  # Constants for drawing
  ROW_HEIGHT = 16
  ROW_SPACE = 12
  ROW_STEP = ROW_HEIGHT+ROW_SPACE

  # Constants for labeling
  LABEL_MARGIN = 8

  # Interaction
  Ruler = 0
  H_adj = gtk.Adjustment()

  # Static variables
  LabelWidth = 200
  TokenCache = 50
  TokenHistory = 1
  
  Center = 0
  PastWidth = 500
  FutureWidth = 500
  MetricTime = False
  TimeScale = 1.0

  TokenSpace = 2

  # Callback containers
  # ContextCallbacks is a dictionary used for registrating extensions to
  # the TREX gui. This dict stores callbacks with string keys. At runtime
  # when a user right-clicks a token, he or she is presented with a the
  # string keys from this dict. When one of the labels is activated,
  # the function in the value part of the dict is called with the arguments:
  #   assembly
  #   token
  ContextCallbacks = {}

  def __init__(self):
    # Initialize data structures
    self.db_core = DbCore()
    self.int_timelines = []
    self.ext_timelines = []
    self.timelines = {}
    self.n_timelines = 0

    # Token structures
    self.all_tokens = {}
    self.token_ticks = {}
    self.token_timelines = {}
    self.tokens_to_remove = []

    # Sorted token structures
    self.started_token_keys = []
    self.planned_token_keys = []

    self.started_token_times = []
    self.planned_token_times = []

    # Drawing variables
    self.needs_redraw = True
    self.timelines_group = None
    self.color = (0,0,0)
    self.hilight_keys = []

    # Initialize tab index (used for keeping track of where this reactor is in the notebook)
    self.tab_index = 0

    # Initialize icon
    self.icon = None

    # Create glade window
    tree = gtk.glade.XML("timeline_window.glade",root="timeline_panel")
    self.w = tree.get_widget("timeline_panel")
    self.w.show_all()

    # Bind the scrolledwindow widgets
    self.timeline_sw = tree.get_widget("timeline_sw")
    self.timeline_label_vp = tree.get_widget("timeline_label_vp")
    self.timeline_label_sw = tree.get_widget("timeline_label_sw")

    # Make scrolled windows share vertical adjustment object for synchronization
    self.v_adj = gtk.Adjustment()
    self.timeline_sw.set_vadjustment(self.v_adj)
    self.timeline_label_sw.set_vadjustment(self.v_adj)

    # Make all windows share the same timeline position
    self.timeline_sw.set_hadjustment(ReactorPanel.H_adj)

    # Bind the drawingarea widgets
    self.timeline_da = tree.get_widget("timeline_da")
    self.timeline_label_da = tree.get_widget("timeline_label_da")

    # Register callbacks for updating the timeline labels and timeline
    self.timeline_label_da.connect("expose-event",self.expose_timeline_label_da)

    self.timeline_da.connect("expose-event",self.expose_timeline_da)
    self.timeline_da.connect("button-press-event", self.on_timeline_click,None)

  # Create timeline structures for all of the timelines in an assembly
  def process_timelines(self, db_core):
    # Save db_core
    self.db_core = db_core

    # Clear timeline vars
    self.n_timelines = 0

    # Remove tokens that were tagged for removal
    for token in self.tokens_to_remove:
      # Get token key
      key = token.key

      if self.all_tokens.has_key(key):
	# Remove from all token map
	del self.all_tokens[key]

	# Remove from sorted lists
	if token.key in self.planned_token_keys:
	  times = self.planned_token_times
	  keys = self.planned_token_keys
	else:
	  times = self.started_token_times
	  keys = self.started_token_keys

	# Get the index in the sorted lists of this token
	sorted_index = keys.index(key)
	# Remove from the sorted lists
	times.pop(sorted_index)
	keys.pop(sorted_index)
      
    # Clear tokens to remove list
    self.tokens_to_remove = []

    # TODO:Clear timelines that are not represented in the assembly

    # Create timeline objects for all timelines
    # This also classifies all timelines as internal or external
    for tl in db_core.int_timelines.values() + db_core.ext_timelines.values():
      # Check if this is a new timeline
      if not self.timelines.has_key(tl.name):
	# Create a new timeline object
	timeline = TimelineSprite(tl)

	# Add the timeline
	self.timelines[timeline.tl.name] = timeline

	if timeline.tl.mode == Timeline.INTERNAL:
	  timeline.reactor_panel = self
	  self.int_timelines.append(timeline)
	else:
	  self.ext_timelines.append(timeline)
      else:
	# Retrieve the timeline object
	timeline = self.timelines[tl.name]
	# Update the object
	timeline.add_tokens(tl.tokens)


      # Add all tokens to this reactor
      ##############################################################

      for new_token in timeline.new_active_tokens:
	# Check if this token existed in a previously viewed tick
	if self.all_tokens.has_key(new_token.key):
	  # Check if this token is planned from the previous tick
	  # This means that it might have started on this one, also it's start time is not yet closed,
	  # so we need to remove it before sorting
	  if new_token.key in self.planned_token_keys:
	    # Get the index in the sorted lists of this token
	    sorted_index = self.planned_token_keys.index(new_token.key)
	    # Remove from the sorted lists
	    self.planned_token_times.pop(sorted_index)
	    self.planned_token_keys.pop(sorted_index)
	  else:
	    # We're travelling backwards
	    # Get the index in the sorted lists of this token
	    sorted_index = self.started_token_keys.index(new_token.key)
	    # Remove from the sorted lists
	    self.started_token_times.pop(sorted_index)
	    self.started_token_keys.pop(sorted_index)
	    pass

	# Store / update this token
	self.all_tokens[new_token.key] = new_token
	# Update last updated tick for this token
	self.token_ticks[new_token.key] = db_core.tick
	# Store this token's timeline
	self.token_timelines[new_token.key] = timeline

	# Insert this token to the appropriate sorted token list
	if new_token.start[0] == new_token.start[1]:
	  keys = self.started_token_keys
	  times = self.started_token_times
	else:
	  keys = self.planned_token_keys
	  times = self.planned_token_times

	# Get insertion indices
	insert_index_start = bisect.bisect_left(times, new_token.start[0])
	insert_index_end = bisect.bisect_right(times, new_token.start[0])

	# Insert token
	if insert_index_start >= len(keys) or new_token.key not in keys[insert_index_start:insert_index_end]:
	  keys.insert(insert_index_start,new_token.key)
	  times.insert(insert_index_start,new_token.start[0])

      ##############################################################

    # Set the row in each timeline 
    row = 0
    for timeline in self.int_timelines + self.ext_timelines:
      timeline.row = row
      row = row + 1

    # Set the number of timelines
    self.n_timelines = row

  # Callback to process click events on the timeline view
  def on_timeline_click(self, widget, event, data):
    # Calculate row
    row = int(event.y/ReactorPanel.ROW_STEP)
    if row > self.n_timelines-1:
      return False

    # Get timeline
    timeline = (self.int_timelines+self.ext_timelines)[row]

    # Do hit test
    token,hit_edge = timeline.get_hit_token(event.x)

    # Process click type
    if event.type == gtk.gdk._2BUTTON_PRESS:
      if event.button == 1:
	if token:
	  if token.key not in self.hilight_keys:
	    self.hilight_keys.append(token.key)
	  else:
	    self.hilight_keys.remove(token.key)
	  self.draw()
    elif event.type == gtk.gdk.BUTTON_PRESS: 
      if event.button == 1:
	pass
      elif event.button == 2:
	# Set the ruler
	if ReactorPanel.Ruler == hit_edge:
	  ReactorPanel.Ruler = 0
	else:
	  ReactorPanel.Ruler = hit_edge
	self.draw()
      elif event.button == 3:
	if token:
	  # Create context menu
	  m = gtk.Menu()

	  # Create info label menu item
	  info = gtk.MenuItem("Timeline: %s\nToken: %s\nKey: %s" % (timeline.tl.name, token.name, str(token.key)),False)
	  info.set_sensitive(False)
	  m.append(info)
	  m.append(gtk.SeparatorMenuItem())

	  # Iterate over extensions
	  for label_str,cb in ReactorPanel.ContextCallbacks.items():
	    menu_ext = gtk.MenuItem(label_str)
	    menu_ext.connect("activate",self.callback_wrapper,cb,self.db_core,token)
	    m.append(menu_ext)

	  # Show the menu, and pop it up
	  m.show_all()
	  m.popup(None, None, None, event.button, event.time, None)


    return False

  # Wrap the callback to keep it from receiving the menuitem
  def callback_wrapper(self,menuitem,cb,db_core,token):
    cb(db_core, token)

  #############################################################################
  # Drawing timelines
  #############################################################################

  # Callback to re-draw a timeline cr when necessary
  def expose_timeline_da(self, widget, event):
    # Create the cairo context
    cr = widget.window.cairo_create()

    # Set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    # Determine if a redraw is necessary
    """
    if self.needs_redraw:
      cr.push_group()
      self.draw_timeline_da(cr)
      self.timelines_group = cr.pop_group()
      self.needs_redraw = False
    
    # Draw the stored timelines group
    cr.set_source(self.timelines_group)
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.fill()
    """
    
    # Draw timelines
    self.draw_timeline_da(cr)

  # Draws the group from scratch
  def draw_timeline_da(self, cr):

    # Get visible width of timeline drawing area
    timeline_da_width = self.timeline_sw.get_allocation().width

    # Clear the image
    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    # Initialize row counter
    row = 0

    # Draw row backgrounds
    row_width = ReactorPanel.PastWidth + ReactorPanel.FutureWidth + timeline_da_width
    for timeline in self.int_timelines + self.ext_timelines:
      row_y = row*ReactorPanel.ROW_STEP + ReactorPanel.ROW_SPACE

      # Draw timeline background
      cr.set_source_rgba(0.8, 0.8, 0.8, 0.7)
      cr.rectangle(0,row_y,row_width,ReactorPanel.ROW_HEIGHT)
      cr.fill()
      row = row+1

    # Draw the execution frontier
    cr.set_source_rgba(0, 0, 0, 0.2)
    cr.rectangle(ReactorPanel.PastWidth + ReactorPanel.Center,0,row_width,(self.n_timelines)*ReactorPanel.ROW_STEP)
    cr.fill()

    # Set global earliest start and latest end
    earliest_tick = float("Inf")
    earliest_start = 0
    latest_end = 0
    latest_tick = 0

    if ReactorPanel.MetricTime:
      token_space = min(2,ReactorPanel.TimeScale/2)
    else:
      token_space = ReactorPanel.TokenSpace
    
    # Reset earliest start and latest end for each reactor
    for tl in self.int_timelines + self.ext_timelines:
      tl.hit_boundaries = [0]
      tl.hit_tokens = [None]
      tl.earliest_tick = float("Inf")
      tl.earliest_start = 0
      tl.latest_end = 0
      tl.latest_tick = 0

    # Iterate over all token keys
    #print "STARTED TOKENS: %d" % len(self.started_token_keys)
    #print "PLANNED TOKENS: %d" % len(self.planned_token_keys)
    for key in self.started_token_keys[::-1] + self.planned_token_keys:

      # Get token
      token = self.all_tokens[key]
      timeline = self.token_timelines[key]
      tick = self.token_ticks[key]
      
      # Get timeline color
      (r,g,b) = timeline.get_color()

      # Skip tokens that are older than the tickbehind and newer than the current tick
      if tick < self.db_core.tick-ReactorPanel.TokenHistory:
	# Check if this is older than the cache
	if tick < self.db_core.tick-ReactorPanel.TokenCache:
	  # Add this token to the removal list
	  self.tokens_to_remove.append(token)
	continue
      elif tick > self.db_core.tick:
	continue

      # Do not draw BaseState (HACK)
      if token.name == "BaseState.Holds":
	pass#continue

      #print "TICK: %d, EARLIEST_START: %d, LATEST_END: %d" % (self.assembly.tick, earliest_start, latest_end)

      # Create the label string, and get the length of the label
      self.set_label_font(cr)
      label_str = "%s" % (token.name.split('.')[1])#, int(str(token.key)))
      _xb,_yb,w_label,_th,_xa,_ya = cr.text_extents(label_str)

      # Create the time bound string and get its length
      self.set_times_font(cr)
      end_str = "[%d, %d]" % (token.start[0], token.end[0])
      _xb,_yb,w_end_str,_th,_xa,_ya = cr.text_extents(end_str)

      # Get the max width of the label
      tok_width_label = max(w_label,w_end_str)
      tok_width_label = tok_width_label + 2*ReactorPanel.LABEL_MARGIN

      tok_x0 = 0
      tok_y0 = 0

      # Switch draw ordering behavior if this token has started or is planned
      if token.start[0] == token.start[1]:
	# Has started
	
	# Initialize token synchronization width
	tok_width_sync = 0

	if token.start[0] < earliest_tick:
	  # Increase spacing if this tick happened before the earliest tick in this view
	  tok_width_sync = ReactorPanel.ROW_HEIGHT/2

	if timeline.earliest_tick > earliest_tick and token.end[0] < earliest_tick:
	  # A token is missing, re-sync to create a space for it
	  timeline.earliest_start = earliest_start

	#print "[%s, %s] earliest: %s (%s)" % (str(token.start[0]),str(token.end[0]), str(earliest_tick), label_str)

	# Calculate the token pixel width
	# Get the width if this token were to be drawn between the latest point on this timeline, and the earliest point for all timelines
	tok_width_sync = tok_width_sync + abs(timeline.earliest_start - earliest_start)

	# Get the larger of the two widths
	tok_width = max(tok_width_label, tok_width_sync)

	if ReactorPanel.MetricTime:
	  tok_width = ReactorPanel.TimeScale*(token.end[0]-token.start[0])

	# Calculate the token end point
	# This is the start of the earliest token on the timeline that this token is being drawn onto
	tok_end = timeline.earliest_start

	# Do not draw token if it ends before the visible window
	#if tok_end < -ReactorPanel.PastWidth:
	#  continue

	# Set the earliest tick for this timeline
	earliest_tick = token.start[0]
	# Set the earliest tick for this timeline
	timeline.earliest_tick = earliest_tick
	# Increment earliest start for this timeline
	timeline.earliest_start = timeline.earliest_start - tok_width
	# Set the new earliest start for all timeline
	earliest_start = min(earliest_start, timeline.earliest_start)

	# Calculate the position top-right corner of the token
	tok_x0 = math.ceil(tok_end)

	#print "TOKEN \"%s\" ADD WIDTH: %d = max( abs( %d - %d ), %d )" % (token.name,tok_width, timeline.earliest_start, earliest_start, tok_width_label)
      else:
	# Is planned

	# Initialize token synchronization width
	tok_width_sync = 0

	if token.end[0] > latest_tick:
	  # Increase spacing if this tick happened before the earliest tick in this view
	  tok_width_sync = ReactorPanel.ROW_HEIGHT/2

	# Calculate the token pixel width
	# Get the width if this token were to be drawn between the latest point on this timeline, and the earliest point for all timelines
	tok_width_sync = tok_width_sync + abs(latest_end - timeline.latest_end)

	# Get the larger of the two widths
	tok_width = max(tok_width_label, tok_width_sync)

	# Calculate the token end point
	# This is the start of the earliest token on the timeline that this token is being drawn onto
	tok_start = timeline.latest_end

	# Do not draw token if it start outside the visible window
	if tok_start < -ReactorPanel.FutureWidth:
	  continue

	# Update latest tick
	latest_tick = token.end[0]
	# Set timeline latest tick
	timeline.latest_tick = latest_tick
	# Increment earliest start for this timelines
	timeline.latest_end = timeline.latest_end + tok_width
	# Set the new earliest start for all timelines
	latest_end = max(latest_end, timeline.latest_end)

	# Calculate the position top-right corner of the token
	tok_x0 = math.ceil(tok_start+tok_width)

      tok_x0 = tok_x0 +ReactorPanel.PastWidth + ReactorPanel.Center
      tok_y0 = ReactorPanel.ROW_STEP*timeline.row + ReactorPanel.ROW_SPACE

      # Store the token hit region
      if token.start[0] == token.start[1]:
	timeline.hit_boundaries.insert(1,tok_x0)
	timeline.hit_tokens.insert(1,token)
      else:
	timeline.hit_boundaries.append(tok_x0)
	timeline.hit_tokens.append(token)

      # Update the edge of the hit region
      timeline.hit_boundaries[0] = ReactorPanel.PastWidth + ReactorPanel.Center + timeline.earliest_start

      # Draw token
      # Set the color for the appropriate reactors
      if key in self.hilight_keys:
	cr.set_source_rgba(1.0, 0.7, 0.07, 1.0)
      elif self.token_ticks[key] < self.db_core.tick:
	cr.set_source_rgba(r,g,b, 0.3)
      else:
	cr.set_source_rgba(r, g, b, 0.7)

      # Draw the token rectangle
      cr.rectangle(tok_x0, tok_y0, -tok_width+token_space, ReactorPanel.ROW_HEIGHT)
      cr.fill()

      # Only draw the labels if there is space for them
      if tok_width-2-ReactorPanel.LABEL_MARGIN > w_label:
	# Draw the token label
	self.set_label_font(cr)
	tx = tok_x0 - w_label - ReactorPanel.LABEL_MARGIN
	ty = tok_y0 + ReactorPanel.ROW_HEIGHT - 4
	cr.move_to(tx,ty)
	cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
	cr.show_text(label_str)

      if tok_width-2-ReactorPanel.LABEL_MARGIN > w_end_str:
	# Draw the time bounds
	self.set_times_font(cr)
	cr.set_source_rgba(0, 0, 0, 0.5)
	tx = tok_x0 - w_end_str - ReactorPanel.LABEL_MARGIN
	ty = tok_y0 + ReactorPanel.ROW_STEP - 3
	cr.move_to(tx,ty)
	cr.show_text(end_str)

    # Draw ruler
    cr.set_source_rgba(0, 0, 0, 0.5)
    cr.rectangle(ReactorPanel.Ruler,0,2,(self.n_timelines)*ReactorPanel.ROW_STEP)
    cr.fill()

    return False

  def set_label_font(self,cr):
    cr.select_font_face(
	"Sans",
	cairo.FONT_SLANT_NORMAL, 
	cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(10)

  def set_times_font(self,cr):
    cr.select_font_face(
	"Monospace",
	cairo.FONT_SLANT_NORMAL, 
	cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(8)

  #############################################################################
  # Drawing timeline labels
  #############################################################################

  def expose_timeline_label_da(self, widget, event):
    # Create the cairo context
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    # Clear the image
    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()
    
    # Determine width needed to show all labels
    max_width = 0
    self.set_label_font(cr)
    for timeline in self.int_timelines + self.ext_timelines:
      # Get extents
      xb,yb,w,h,xa,ya = cr.text_extents(timeline.tl.name)
      max_width = max(w,max_width)

    # Set the width
    ReactorPanel.LabelWidth = max(ReactorPanel.LabelWidth, max_width + (2*ReactorPanel.LABEL_MARGIN))

    # Draw rows
    row = 0
    for timeline in self.int_timelines + self.ext_timelines:
      y = row*ReactorPanel.ROW_STEP + ReactorPanel.ROW_SPACE

      # Get color based on parent
      (r,g,b) = timeline.get_color()
      cr.set_source_rgba(r, g, b, 1.0)
      cr.rectangle(0,y,1000,ReactorPanel.ROW_HEIGHT)
      cr.fill()

      # Determine extents of text string
      xb,yb,w,h,xa,ya = cr.text_extents(timeline.tl.name)

      tx = ReactorPanel.LabelWidth - w - ReactorPanel.LABEL_MARGIN
      ty = y+ReactorPanel.ROW_HEIGHT-4
      cr.move_to(tx,ty)
      cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
      cr.show_text(timeline.tl.name)

      row = row+1

    # Resize drawing area for the current number of tokens
    win_height_px = ReactorPanel.ROW_SPACE + ReactorPanel.ROW_STEP*self.n_timelines
    self.timeline_da.set_size_request(int(ReactorPanel.PastWidth + ReactorPanel.FutureWidth),win_height_px)

    # Resize drawing area for token width
    self.timeline_label_da.set_size_request(int(ReactorPanel.LabelWidth),win_height_px)
    self.timeline_label_sw.set_size_request(int(ReactorPanel.LabelWidth+20),-1)

    return False

  def draw(self, redraw = True):
    # Set the needs_redraw flag
    self.needs_redraw = redraw

    # Redraw the timelines drawingarea
    if self.timeline_da.window:
      # Refresh view
      alloc = self.timeline_da.get_allocation()
      rect = gtk.gdk.Rectangle(0, 0, alloc.width, alloc.height)
      self.timeline_da.window.invalidate_rect(rect, True)
      self.timeline_da.window.process_updates(True)

    # Redraw the labels drawingarea
    if self.timeline_label_da.window:
      # Refresh view
      alloc = self.timeline_label_da.get_allocation()
      rect = gtk.gdk.Rectangle(0, 0, alloc.width, alloc.height)
      self.timeline_label_da.window.invalidate_rect(rect, True)
      self.timeline_label_da.window.process_updates(True)

##############################################################################
# TimelineWindow
#   This is a GTK widget container class that shows a user a set of partial
#   plans from the reactors in a given TREX agent.
##############################################################################

class TimelineWindow():
  def __init__(self):
    # Initialize structures
    self.reactor_panels = {}
    self.reactor_indices = {}
    self.colors = [(0,0,0)]

    # Create glade window
    self.tree = gtk.glade.XML("timeline_window.glade")
    self.w = self.tree.get_widget("timeline_window")
    self.w.set_title("TREX Timelines")

    # Add references to all underscored widgets
    for w in self.tree.get_widget_prefix('_'):
      name = w.get_name()[1:]
      # Make sure we don't clobber existing attributes
      assert not hasattr(self, name)
      setattr(self, name, w)

    # Remove template page from notebook
    self.reactor_nb.remove_page(0)

    # add tabs
    random.seed(4)

    #self.time_scale_slider.connect("change-value",self.redraw_viewport)
    #self.viewport_da.connect("expose-event",self.expose_viewport)

    # Connect menu view check
    self.show_view_options_menu_item.connect("toggled",self.on_toggle_view_controls)

    # connect view controls
    self.past_width_spin.connect("value-changed",self.on_change_view_controls)
    self.center_spin.connect("value-changed",self.on_change_view_controls)
    self.future_width_spin.connect("value-changed",self.on_change_view_controls)
    self.metric_time_check.connect("toggled",self.on_change_view_controls)
    self.time_scale_spin.connect("value-changed",self.on_change_view_controls)

    self.token_cache_spin.connect("value-changed",self.on_change_view_controls)
    self.token_history_spin.connect("value-changed",self.on_change_view_controls)

    self.w.show()

  #############################################################################
  # UI Event handlers
  #############################################################################

  # Callback to hide and show the view controls
  def on_toggle_view_controls(self,widget):
    if self.show_view_options_menu_item.get_active():
      self.view_options_box.show()
    else:
      self.view_options_box.hide()

  # Callback to propagate the view control settings into the reactor panel and redraw
  def on_change_view_controls(self,widget):
    # Get control values
    ReactorPanel.PastWidth = self.past_width_spin.get_value()
    ReactorPanel.FutureWidth = self.future_width_spin.get_value()
    ReactorPanel.Center = self.center_spin.get_value()
    ReactorPanel.MetricTime = self.metric_time_check.get_active()
    ReactorPanel.TimeScale = self.time_scale_spin.get_value()

    ReactorPanel.TokenCache = self.token_cache_spin.get_value()
    ReactorPanel.TokenHistory = self.token_history_spin.get_value()

    self.draw_active_reactor()

  # Set the status text
  def set_status(self,text):
    self.status_text = text
    self.statusbar.pop(0)
    self.statusbar.push(0,self.status_text)

  #############################################################################
  # Data manipulation
  #############################################################################

  # Load new assemblies 
  def set_db_cores(self,db_cores,selected_reactor_name):
    # Add and remove reactors as necessary
    for reactor_name,db_core in db_cores.items():
      # Check if this reactor exists
      if not self.reactor_panels.has_key(reactor_name):
	self.add_reactor(reactor_name)
      # Set this reactor's assembly
      self.reactor_panels[reactor_name].process_timelines(db_core)

    # Remove reactors that were not updated
    removed_reactors = [rname for rname in self.reactor_panels.keys() if rname not in db_cores.keys()]
    for reactor_name in removed_reactors:
      self.rem_reactor(reactor_name)

    # Determine external timeline parents
    for reactor_panel in self.reactor_panels.values():
      for timeline in reactor_panel.ext_timelines:
	for tl_parent in self.reactor_panels.values():
	  # Only compare to reactors that aren't the current one
	  if tl_parent != reactor_panel:
	    # Generate a list of the internal timeline names for this candidate parent
	    tl_names = [tl.tl.name for tl in tl_parent.int_timelines]
	    if timeline.tl.name in tl_names:
	      # Set this timeline's reactor panel to it's parent
	      timeline.reactor_panel = tl_parent
		
    # Redraw the active reactor
    self.draw_active_reactor()

  #############################################################################
  # Methods for manipulating reactor views
  #############################################################################

  # Add a reactor (and create a new tab)
  def add_reactor(self,reactor_name):
    # Create a new timeline panel
    tp = ReactorPanel()
    self.reactor_panels[reactor_name] = tp

    # Create a new label
    tree = gtk.glade.XML("timeline_window.glade",root="tab_label")
    label = tree.get_widget("tab_label")
    icon = tree.get_widget("tab_icon")
    text = tree.get_widget("tab_text")

    # Set the label text
    text.set_text(reactor_name)

    # Store a reference to the icon widget
    tp.icon = icon

    # Append the reactor to the notebook
    tp.tab_index = self.reactor_nb.insert_page(tp.w,label)
    self.reactor_nb.set_menu_label_text(tp.w,reactor_name)

    # Store the index
    self.reactor_indices[tp.tab_index] = tp

    # Create timelines for this reactor
    

    # Update the colors
    self.update_icons()

    # Show all of the label sub-widgets
    label.show_all()

  # Remove a reactor (and delete its tab)
  def rem_reactor(self,reactor_name):
    # Get page index
    tab_index = self.reactor_panels[reactor_name].tab_index

    # Remove notebook page
    self.reactor_nb.remove_page(tab_index)

    # Remove from reactor list
    del self.reactor_panels[reactor_name]

    # Update tab indices for all other reactors
    for reactor_panel in self.reactor_panels.values():
      if reactor_panel.tab_index > tab_index:
	# Decrease tab index by one
	reactor_panel.tab_index = reactor_panel.tab_index - 1
	# Update the index
	self.reactor_indices[reactor_panel.tab_index] = reactor_panel

    # Update the colors
    self.update_icons()

  # Draw the selected reactor
  def draw_active_reactor(self):
    pid = self.reactor_nb.get_current_page()
    if self.reactor_indices.has_key(pid):
      self.reactor_indices[pid].draw()

  # Generate a random color that is some distance from all other colors in rgb space
  def gen_rand_color(self):
    r = 0
    g = 0
    b = 0
    min_dist = 0
    while min_dist < 0.1:
      r = random.random()
      g = random.random()
      b = random.random()

      dist = float("Inf")
      for c in self.colors:
	dist = min(dist,(pow(r-c[0],2)+pow(g-c[1],2)+pow(b-c[2],2)))

      min_dist = dist

    return (r,g,b)

  # Update the icon pixbufs with new colors based on the number of reactors
  def update_icons(self):
    # Get total number of reactors
    n_reactors = len(self.reactor_panels)

    # Initialize reactor count
    for reactor_panel in self.reactor_panels.values():
      icon = reactor_panel.icon
      canvas = gtk.gdk.Pixmap(None,16,16,24)

      vis = gtk.gdk.visual_get_system()
      cmap = gtk.gdk.Colormap(vis,True)
      canvas.set_colormap(cmap)
      cr = canvas.cairo_create()

      # set a clip region 
      cr.rectangle(0,0,16,16)
      cr.clip()

      # Generate the color
      rgb = colorsys.hsv_to_rgb(float(reactor_panel.tab_index)/float(n_reactors),0.4,0.5)
      #rgb = self.gen_rand_color()
      reactor_panel.color = rgb
      cr.set_source_rgba(rgb[0], rgb[1], rgb[2], 1.0)
      cr.paint()

      # Get pixbuf from drawable
      pixbuf = gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,16,16)
      pixbuf.get_from_drawable(canvas,cmap,0,0,0,0,16,16)

      icon.set_from_pixbuf(pixbuf)

  #############################################################################
  # Extension API
  #############################################################################

  def register_context_extension(self,label_str,cb):
    # Add or replace callback in dictionary
    ReactorPanel.ContextCallbacks[label_str] = cb

  def unregister_context_extension(self,label_str,cb):
    # Remove callback from dictionary
    if ReactorPanel.has_key(label_str):
      del ReactorPanel[label_str]


# Unit tests
class TestTokenNetworkWindow(unittest.TestCase):
  # Create the gtk thread and window structure
  def setUp(self):
    # Create a new timeline window
    self.timeline_window = TimelineWindow()
    
  # Destroy window and kill gtk
  def tearDown(self):
    print "Killing The window..."
    self.timeline_window.w.destroy()
    time.sleep(5)

  # Test the auto-redraw of a network
  def test_window(self):
    gtk.gdk.threads_init()
    self.timeline_window.w.connect("destroy",gtk.main_quit)
    test_thread = threading.Thread(target=self.thread_test_window)
    test_thread.start()
    gtk.main()

  def thread_test_window(self):
    print "Started test thread..."
    time.sleep(2)

    print "Constructing test assembly..."
    from assembly import Assembly,construct_test_assembly

    # Create assemblies
    assemblies = {}
    assemblies["test"] = construct_test_assembly()

    print "Adding reactors..."
    gtk.gdk.threads_enter()
    self.timeline_window.add_reactor("doorman")
    self.timeline_window.add_reactor("recharger")
    self.timeline_window.add_reactor("state_estimator")
    self.timeline_window.add_reactor("mechanism_control")
    self.timeline_window.add_reactor("master")
    self.timeline_window.add_reactor("navigator")
    self.timeline_window.add_reactor("driver")
    self.timeline_window.add_reactor("writer")
    gtk.gdk.threads_leave()
    time.sleep(1)
    gtk.gdk.threads_enter()
    self.timeline_window.rem_reactor("state_estimator")
    gtk.gdk.threads_leave()
    time.sleep(1)
    gtk.gdk.threads_enter()
    self.timeline_window.rem_reactor("writer")
    gtk.gdk.threads_leave()
    time.sleep(1)
    gtk.gdk.threads_enter()
    self.timeline_window.rem_reactor("doorman")
    gtk.gdk.threads_leave()

    time.sleep(30)

if __name__ == '__main__':
  unittest.main()
