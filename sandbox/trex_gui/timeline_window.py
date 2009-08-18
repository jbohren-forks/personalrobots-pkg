#!/usr/bin/env python

# System modules
import sys,os
import random
import colorsys
import unittest
import threading, time
import math

import cairo, pangocairo
import gtk, gtk.glade

# TREX modules
from assembly import Assembly,Entity,Rule,Token,Slot,Variable
from token_network import TokenNetwork
from token_network_filter import TokenNetworkFilter

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

class Timeline():
  # Timeline constants
  INTERNAL, EXTERNAL = range(2)

  def __init__(self,obj):
    # Store this object
    self.obj = obj

    # Store the reactor that this timeline belongs to
    self.reactor_panel = None

    # Store a list of active tokens
    self.tokens = []

    # Internal timelines must be declared as internal or external
    for var in obj.vars:
      # Check the mode value
      if var.name[-5:] == ".mode":
	# Internal timeline
	if -1 != var.values.find("Internal"):
	  self.mode = Timeline.INTERNAL
	else:
	  self.mode = Timeline.EXTERNAL
    
    # Copy only active tokens onto the token list
    for token in obj.tokens:
      if token.slot_index == 0:
	self.tokens.append(token)

    # Sort the list of tokens
    self.tokens.sort(lambda a,b: int(b.start[0] - a.start[0]))


  
  # Get the color for this timeline
  def get_color(self):
    if self.reactor_panel:
      rgb = self.reactor_panel.color
    else:
      # No parent
      rgb = (0.3,0.3,0.3)

    return rgb


class ReactorPanel():
  # Constants for drawing
  ROW_HEIGHT = 16
  ROW_SPACE = 8
  ROW_STEP = ROW_HEIGHT+ROW_SPACE

  # Constants for labeling
  LABEL_MARGIN = 8

  # Static variables
  LabelWidth = 200

  def __init__(self):
    # Initialize data structures
    self.assembly = Assembly()
    self.int_timelines = []
    self.ext_timelines = []
    self.n_timelines = 0

    # Drawing variables
    self.color = (0,0,0)

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

    # Bind the drawingarea widgets
    self.timeline_da = tree.get_widget("timeline_da")
    self.timeline_label_da = tree.get_widget("timeline_label_da")

    # Register callbacks for updating the timeline labels and timeline
    self.timeline_label_da.connect("expose-event",self.expose_timeline_label_da)

    self.timeline_da.connect("expose-event",self.expose_timeline_da)
    self.timeline_da.connect("button-press-event", self.on_timeline_click,None)

  # Create timeline structures for all of the timelines in an assembly
  def process_timelines(self, assembly):
    # Clear timeline vars
    self.n_timelines = 0
    self.int_timelines = []
    self.ext_timelines = []

    # Create timeline objects for all timelines
    # This also classifies all timelines as internal or external
    for object in assembly.objects.values():
      # Timelines must have tokens
      if len(object.tokens) > 0:
	# Create a new timeline object
	timeline = Timeline(object)
	if timeline.mode == Timeline.INTERNAL:
	  timeline.reactor_panel = self
	  self.int_timelines.append(timeline)
	else:
	  self.ext_timelines.append(timeline)

    # Set the number of timelines
    self.n_timelines = len(self.int_timelines) + len(self.ext_timelines)

  # Callback to process click events on the timeline view
  def on_timeline_click(self, widget, event, data):
    if event.button == 2:
      pass
    elif event.button == 3:
      # Calculate row
      row = int(event.y/ReactorPanel.ROW_STEP)

      if row > self.n_timelines-1:
	return False

      m = gtk.Menu()
      pos = gtk.MenuItem("%s Pixel: %d" % ((self.int_timelines+self.ext_timelines)[row], event.x),False)
      pos.set_sensitive(False)
      t_net = gtk.MenuItem("Token network")
      t_vars = gtk.MenuItem("Token variables")
      m.append(pos)
      m.append(t_net)
      m.append(t_vars)
      m.show_all()
      m.popup(None, None, None, event.button, event.time, None)

    return False

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

    # Clear the image
    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    # Initialize row counter
    row = 0

    # Print rows
    for timeline in self.int_timelines + self.ext_timelines:
      y = row*ReactorPanel.ROW_STEP + ReactorPanel.ROW_SPACE

      # Draw timeline background
      cr.set_source_rgba(0, 0, 0, 0.1)
      cr.rectangle(0,y,1000,ReactorPanel.ROW_HEIGHT)
      cr.fill()

      # Get timeline color
      (r,g,b) = timeline.get_color()

      # Draw tokens
      x = 500
      self.set_font(cr)
      for token in timeline.tokens:
	# Determine label width
	name = token.name.split(".")[1]
	xb,yb,tw,th,xa,ya = cr.text_extents(name)
	
	# Draw box
	bw = tw+(2*ReactorPanel.LABEL_MARGIN)
	cr.set_source_rgba(r, g, b, 1.0)
	cr.rectangle(x,y,-bw,ReactorPanel.ROW_HEIGHT)
	cr.fill()

	tx = x -tw - ReactorPanel.LABEL_MARGIN
	ty = y+ReactorPanel.ROW_HEIGHT-4

	# Draw label
	cr.move_to(tx,ty)
	cr.set_source_rgba(1.0, 1.0, 1.0, 0.5)
	cr.show_text(name)

	x = x-bw-1


      # Increment row counter
      row = row+1


    return False

  # Draw a token on a given row
  def draw_token(self,cr,row,start,end):
    pass

  def set_font(self,cr):
    cr.select_font_face(
	"Sans",
	cairo.FONT_SLANT_NORMAL, 
	cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(13)

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
    self.set_font(cr)
    for timeline in self.int_timelines + self.ext_timelines:
      # Get extents
      xb,yb,w,h,xa,ya = cr.text_extents(timeline.obj.name)
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
      xb,yb,w,h,xa,ya = cr.text_extents(timeline.obj.name)

      tx = ReactorPanel.LabelWidth - w - ReactorPanel.LABEL_MARGIN
      ty = y+ReactorPanel.ROW_HEIGHT-4
      cr.move_to(tx,ty)
      cr.set_source_rgba(1.0, 1.0, 1.0, 0.5)
      cr.show_text(timeline.obj.name)

      row = row+1

    # Resize drawing area for the current number of tokens
    win_height_px = ReactorPanel.ROW_SPACE + ReactorPanel.ROW_STEP*self.n_timelines
    self.timeline_da.set_size_request(1000,win_height_px)

    # Resize drawing area for token width
    self.timeline_label_da.set_size_request(int(ReactorPanel.LabelWidth),win_height_px)
    self.timeline_label_sw.set_size_request(int(ReactorPanel.LabelWidth+20),-1)

    return False

  def draw(self):
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

    self.time_scale_slider.connect("change-value",self.redraw_viewport)
    self.viewport_da.connect("expose-event",self.expose_viewport)

    self.w.show()

  #############################################################################
  # Data manipulation
  #############################################################################

  # Load new assemblies 
  def set_assemblies(self,assemblies,selected_reactor_name):
    # Add and remove reactors as necessary
    for reactor_name,assembly in assemblies.items():
      # Check if this reactor exists
      if not self.reactor_panels.has_key(reactor_name):
	self.add_reactor(reactor_name)
      # Set this reactor's assembly
      self.reactor_panels[reactor_name].process_timelines(assembly)

    # Remove reactors that were not updated
    removed_reactors = [rname for rname in self.reactor_panels.keys() if rname not in assemblies.keys()]
    for reactor_name in removed_reactors:
      self.rem_reactor(reactor_name)

    # Determine external timeline parents
    for reactor_panel in self.reactor_panels.values():
      for timeline in reactor_panel.ext_timelines:
	for tl_parent in self.reactor_panels.values():
	  # Only compare to reactors that aren't the current one
	  if tl_parent != reactor_panel:
	    # Generate a list of the internal timeline names for this candidate parent
	    tl_names = [tl.obj.name for tl in tl_parent.int_timelines]
	    if timeline.obj.name in tl_names:
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
  # Methods for drawing the viewport
  # The viewport is the widget used to specify what part of the history can be
  # seen
  #############################################################################

  # Callback when the viewport changes size
  def expose_viewport(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    self.draw_viewport(cr)

    return False

  # Method to manually redraw the viewport
  def redraw_viewport(self,widget, event, value):
    cr = self.viewport_da.window.cairo_create()
    self.draw_viewport(cr)

  # Draw the viewport based on the current view and time scale
  def draw_viewport(self, context):
    context.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    context.paint()

    rect = self.viewport_da.get_allocation()
    x = rect.x + rect.width / 2
    y = rect.y + rect.height / 2

    radius = self.time_scale_slider.get_value()#min(rect.width / 2, rect.height / 2) - 5

    # clock back
    context.arc(x, y, radius, 0, 2 * math.pi)
    context.set_source_rgb(1, 1, 1)
    context.fill_preserve()
    context.set_source_rgb(0, 0, 0)
    context.stroke()

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
