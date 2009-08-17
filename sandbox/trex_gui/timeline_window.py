#!/usr/bin/env python

# System modules
import sys,os
import random
import colorsys
import unittest
import threading, time
import math
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

class ReactorPanel():
  # Constants for drawing
  ROW_HEIGHT = 16
  ROW_SPACE = 8
  ROW_STEP = ROW_HEIGHT+ROW_SPACE

  def __init__(self):
    # Create glade window
    tree = gtk.glade.XML("timeline_window.glade",root="timeline_panel")
    self.w = tree.get_widget("timeline_panel")
    self.w.show_all()

    # Bind the scrolledwindow widgets
    self.timeline_sw = tree.get_widget("timeline_sw")
    self.timeline_label_sw = tree.get_widget("timeline_label_sw")

    # Make scrolled windows share vertical adjustment object for synchronization
    v_adj = gtk.Adjustment()
    self.timeline_sw.set_vadjustment(v_adj)
    self.timeline_label_sw.set_vadjustment(v_adj)

    # Bind the drawingarea widgets
    self.timeline_da = tree.get_widget("timeline_da")
    self.timeline_label_da = tree.get_widget("timeline_label_da")

    # Register callbacks for updating the timeline labels and timeline
    self.timeline_label_da.connect("expose-event",self.expose_timeline_label)
    self.timeline_da.connect("expose-event",self.expose_timeline)
    self.timeline_da.connect("button-press-event", self.on_timeline_click,None)


    # Playing around
    self.timeline_da.set_size_request(2200,500)
    self.timeline_label_da.set_size_request(200,500)

  # Callback to process click events on the timeline view
  def on_timeline_click(self, widget, event, data):
    if(event.button != 3):
      return False

    # Calculate row
    row = int(event.y/ReactorPanel.ROW_STEP)

    m = gtk.Menu()
    pos = gtk.MenuItem("Clicked Row: %d Pixel: %d" % (row, event.x))
    pos.set_sensitive(False)
    t_net = gtk.MenuItem("Token network")
    t_vars = gtk.MenuItem("Token variables")
    m.append(pos)
    m.append(t_net)
    m.append(t_vars)
    m.show_all()
    m.popup(None, None, None, event.button, event.time, None)
    return False

  # Callback to re-draw a timeline cr when necessary
  def expose_timeline(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    # Print rows
    cr.set_source_rgba(0, 0, 0, 0.1)

    for row in range(23):
      cr.rectangle(0,row*ReactorPanel.ROW_STEP+ReactorPanel.ROW_SPACE,1000,ReactorPanel.ROW_HEIGHT)
      cr.fill()

    return False

  def expose_timeline_label(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    return False

  def draw_token(self,cr,row,start,end):
    pass

##############################################################################
# TimelineWindow
#   This is a GTK widget container class that shows a user a set of partial
#   plans from the reactors in a given TREX agent.
##############################################################################

class TimelineWindow():
  def __init__(self):
    # Initialize structures
    self.reactor_names = []
    self.reactor_icons = []
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

    # add tabs
    random.seed(4)
    self.add_reactor("doorman")
    self.add_reactor("recharger")
    self.add_reactor("state_estimator")
    self.add_reactor("mechanism_control")
    self.add_reactor("master")
    self.add_reactor("navigator")
    self.add_reactor("driver")
    self.add_reactor("writer")

    self.time_scale_slider.connect("change-value",self.redraw_viewport)
    self.viewport_da.connect("expose-event",self.expose_viewport)

    self.w.show()

  #############################################################################
  # Methods for manipulating reactor views
  #############################################################################

  # Add a reactor (and create a new tab)
  def add_reactor(self,reactor_name):
    # Create a new timeline panel
    tp = ReactorPanel()

    # Create a new label
    tree = gtk.glade.XML("timeline_window.glade",root="tab_label")
    label = tree.get_widget("tab_label")
    icon = tree.get_widget("tab_icon")
    text = tree.get_widget("tab_text")

    # Append the reactor name to the reactor name list
    self.reactor_names.append(reactor_name)

    # Set the label text
    text.set_text(reactor_name)

    # Store a reference to the icon widget
    self.reactor_icons.append(icon)
    self.update_icons()

    # Append the reactor to the notebook
    self.reactor_nb.append_page(tp.w,label)
    self.reactor_nb.set_menu_label_text(tp.w,reactor_name)

    # Show all of the label sub-widgets
    label.show_all()

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
    n_reactors = len(self.reactor_names)
    for ii in range(n_reactors):
      icon = self.reactor_icons[ii]
      canvas = gtk.gdk.Pixmap(None,16,16,24)

      vis = gtk.gdk.visual_get_system()
      cmap = gtk.gdk.Colormap(vis,True)
      canvas.set_colormap(cmap)
      cr = canvas.cairo_create()

      # set a clip region 
      cr.rectangle(0,0,16,16)
      cr.clip()

      # Generate the color
      rgb = colorsys.hsv_to_rgb(float(ii)/float(n_reactors),0.4,0.5)
      #rgb = self.gen_rand_color()
      self.colors.append(rgb)
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
        
def main():
  timeline_window = TimelineWindow()
  timeline_window.w.connect("destroy",gtk.main_quit)
  gtk.main()

if __name__ == "__main__":
  main()
