#!/usr/bin/env python

# System modules
import sys,os
import unittest
import threading, time
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

class TokenNetworkFilterWindow():
  def __init__(self,token_network_filter=TokenNetworkFilter()):
    # Set up token network filter
    self.token_network_filter = token_network_filter
    self.set_token_network_filter(token_network_filter)

    # Create glade window
    tree = gtk.glade.XML("token_network_filter_window.glade")
    self.w = tree.get_widget("token_network_filter_window")
    self.w.set_title("Search Filters")

    # Add references to all widgets
    for w in tree.get_widget_prefix('_'):
      name = w.get_name()[1:]
      # Make sure we don't clobber existing attributes
      assert not hasattr(self, name)
      setattr(self, name, w)

    # Create liststore model
    self.create_model()

    # Create tree view
    self.filter_view.set_model(self.store)
    self.filter_view.connect('key_press_event', self.on_key_press)
    self.create_columns(self.filter_view)

    self.add_but.connect("clicked",self.add_filter)
    self.rep_but.connect("clicked",self.replace_filter)

    self.filter_entry.set_activates_default(True)
    self.w.set_default(self.rep_but)

    self.w.show()

  ############################################################################
  # Construction utilities
  ############################################################################

  # Initialize filter list
  def create_model(self):
    self.store = gtk.ListStore(bool, str)

  # Create list view columns
  def create_columns(self, treeView):
    # Create checkbox column
    cell = gtk.CellRendererToggle()
    cell.set_property( 'activatable', True )
    cell.connect_object( 'toggled', self.on_set_include, self.store )

    column = gtk.TreeViewColumn("On", cell)
    column.set_sort_column_id(0)    
    column.add_attribute( cell, 'active', 0)

    treeView.append_column(column)

    # Create pattern column
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Pattern", rendererText, text=1)
    column.set_sort_column_id(1)
    treeView.append_column(column)

  # Associate a token_network_filter with this window
  def set_token_network_filter(self,token_network_filter):
    self.token_network_filter = token_network_filter

  ############################################################################
  # Filter manipulation
  ############################################################################

  # Add a filter to the list, and to the network_filter
  def add_filter(self,widget):
    filter = self.filter_entry.get_text()
    self.store.append([ True, filter])
    # Add the filter to the actual token network filter
    self.token_network_filter.add_filter(filter)

  # Remove a filter from the list and the network_filter
  def rem_filter(self, iter):
    # Get the filter tex
    filter = self.store.get_value(iter,1)
    self.store.remove(iter)
    # Remove the filter from the token network filter
    self.token_network_filter.rem_filter(filter)

  # Remove the selected filter
  def remove_selected_filter(self):
    selection = self.filter_view.get_selection()
    model, iter = selection.get_selected()
    self.rem_filter(iter)

  # Replace the currently selected filter
  def replace_filter(self,widget):
    # Remove selected or first filter
    selection = self.filter_view.get_selection()
    model, iter = selection.get_selected()
    if not iter:
      iter = self.store.get_iter_first()
    if iter:
      self.rem_filter(iter)
    # Add the new filter
    self.add_filter(widget)

  ############################################################################
  # Event Handlers
  ############################################################################

  # Callback for processing key presses in the filter list
  def on_key_press(self,widget,event):
    keyname = gtk.gdk.keyval_name(event.keyval)
    if keyname == "BackSpace" or keyname == "Delete":
      self.remove_selected_filter()

  # Process toggling the inclusion of a filter
  def on_set_include(self, model, row):
    model[row][0] = not model[row][0]
    if model[row][0]:
      self.token_network_filter.add_filter(model[row][1])
    else:
      self.token_network_filter.rem_filter(model[row][1])

# Testing utilities
class GtkTester():
  def spawn_gtk_thread(self):
    # Spawn the window
    gtk_thread = threading.Thread(target=self.gtk_thread)
    gtk_thread.start()

  def gtk_thread(self):
    # Spawn a thread to run gtk in
    print "Spawning gtk thread..."
    self.filter_window.w.connect("destroy",gtk.main_quit)
    gtk.main()

# Unit tests
class TestTokenNetworkFilterWindow(unittest.TestCase,GtkTester):
  # Create the gtk thread and window structure
  def setUp(self):
    # Initialize GTK Python threading functionality
    gtk.gdk.threads_init()
    # Create a new db reader window
    self.filter_window = TokenNetworkFilterWindow()
    # Spawn the window
    self.spawn_gtk_thread()
    
  # Destroy window and kill gtk
  def tearDown(self):
    print "Killing The window..."
    self.filter_window.w.destroy()
    time.sleep(5)

  # Test basic addition and removal of filters
  def test_push_network(self):
    from assembly import construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create token network
    token_network = TokenNetwork()
    token_network.set_assembly(assembly)

    # Create token network filter
    token_network_filter = TokenNetworkFilter(token_network)
    
    # Attach token network window to token network
    self.filter_window.set_token_network_filter(token_network_filter)

    print "Adding filter..."
    self.filter_window.filter_entry.set_text("merged")
    self.filter_window.add_but.emit("clicked")

    time.sleep(5)

if __name__ == '__main__':
  unittest.main()
