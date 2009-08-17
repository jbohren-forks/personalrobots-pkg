#!/usr/bin/env python

# System modules
import sys,os
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
# TokenNetworkFilterWindow
#   This class is a GTK window that provides an interface to a
#   TokenNetworkFilter.
##############################################################################

class ReactorPanel():
  def __init__(self):
    # Create glade window
    tree = gtk.glade.XML("timeline_window.glade",root="timeline_panel")
    self.w = tree.get_widget("timeline_panel")
    self.w.show_all()

    self.timeline_da = tree.get_widget("timeline_da")
    self.timeline_label_da = tree.get_widget("timeline_label_da")


    self.timeline_label_da.connect("expose-event",self.expose_timeline_label)
    self.timeline_da.connect("expose-event",self.expose_timeline)

  def expose_timeline(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    return False

  def expose_timeline_label(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
    cr.paint()

    return False


class TimelineWindow():
  def __init__(self):
    # Initialize structures
    self.reactor_names = []
    self.reactor_icons = []

    # Create glade window
    self.tree = gtk.glade.XML("timeline_window.glade")
    self.w = self.tree.get_widget("timeline_window")
    self.w.set_title("TREX Timelines")

    # Add references to all widgets
    for w in self.tree.get_widget_prefix('_'):
      name = w.get_name()[1:]
      # Make sure we don't clobber existing attributes
      assert not hasattr(self, name)
      setattr(self, name, w)

    # add tabs
    self.add_reactor("doorman")
    self.add_reactor("recharger")
    self.add_reactor("state_estimator")
    self.add_reactor("master")

    self.time_scale_slider.connect("change-value",self.redraw_viewport)
    self.viewport_da.connect("expose-event",self.expose_viewport)

    self.w.show()

  #############################################################################
  # Methods for manipulating reactor views
  #############################################################################

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

    # Show all of the label sub-widgets
    label.show_all()

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
      rgb = colorsys.hsv_to_rgb(float(ii)/float(n_reactors),0.5,0.5)
      cr.set_source_rgba(rgb[0], rgb[1], rgb[2], 1.0)
      cr.paint()

      # Get pixbuf from drawable
      pixbuf = gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,16,16)
      pixbuf.get_from_drawable(canvas,cmap,0,0,0,0,16,16)

      icon.set_from_pixbuf(pixbuf)




  #############################################################################
  # Methods for drawing the viewport
  #############################################################################

  def expose_viewport(self, widget, event):
    cr = widget.window.cairo_create()

    # set a clip region for the expose event
    cr.rectangle(event.area.x, event.area.y,event.area.width, event.area.height)
    cr.clip()

    self.draw_viewport(cr)

    return False

  def redraw_viewport(self,widget, event, value):
    cr = self.viewport_da.window.cairo_create()
    self.draw_viewport(cr)

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
