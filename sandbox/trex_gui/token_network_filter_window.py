#!/usr/bin/env python

# System modules
import sys,os
import unittest
import threading, time
import gtk

# TREX modules
from assembly import Assembly,Entity,Rule,Token,Slot,Variable
from token_network import TokenNetwork
from token_network_filter import TokenNetworkFilter

##############################################################################
# TokenNetworkFilterWindow
#   This class is a GTK window that provides an interface to a
#   TokenNetworkFilter.
##############################################################################

class TokenNetworkFilterWindow(gtk.Window):
  def __init__(self,token_network_filter=TokenNetworkFilter()):
    super(TokenNetworkFilterWindow, self).__init__()
    self.set_title("Search Filters")
    self.set_size_request(400, 300)
    self.set_position(gtk.WIN_POS_CENTER)

    self.token_network_filter = token_network_filter
    self.set_token_network_filter(token_network_filter)

    vbox = gtk.VBox(False, 8)
    ###################################################
    sw = gtk.ScrolledWindow()
    sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
    sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)


    # Create liststore model
    self.create_model()

    self.filter_view = gtk.TreeView(self.store)
    #self.filter_view.connect("row-activated", self.on_activated)
    self.filter_view.connect('key_press_event', self.on_key_press)
    self.filter_view.set_rules_hint(True)
    sw.add(self.filter_view)

    self.create_columns(self.filter_view)
    vbox.pack_start(sw, True, True, 0)

    ################################################
    filter_hbox = gtk.HBox(False,4)

    self.add_but = gtk.Button("Add")
    self.add_but.connect("clicked",self.add_filter)

    self.rep_but = gtk.Button("Replace")
    self.rep_but.connect("clicked",self.replace_filter)
    self.rep_but.set_flags(gtk.CAN_DEFAULT)

    self.filter_entry = gtk.Entry()
    self.filter_entry.set_activates_default(True)
    self.set_default(self.rep_but)


    filter_hbox.pack_start(self.filter_entry, True, True, 0)
    filter_hbox.pack_start(self.rep_but, False, False, 0)
    filter_hbox.pack_start(self.add_but, False, False, 0)

    vbox.pack_start(filter_hbox, False, False, 4)
    
    #################################################

    opt_frame = gtk.Frame("Options")
    opt_table = gtk.Table(2,1,homogeneous=False)

    ignore_check = gtk.CheckButton("Ignore Case ")
    halign = gtk.Alignment(1,0.5,0,0.5)
    halign.add(ignore_check)
    opt_table.attach(halign, 0,1,0,1, gtk.FILL, gtk.FILL,0,0)

    regex_check = gtk.CheckButton("Use Regex")
    halign = gtk.Alignment(1,0.5,0,0.5)
    halign.add(regex_check)
    opt_table.attach(halign, 1,2,0,1, gtk.FILL, gtk.FILL,0,0)


    padding = gtk.Alignment(1.0,0.5,1,1)
    padding.set_padding(4,4,4,4)
    padding.add(opt_table)
    opt_frame.add(padding)

    vbox.pack_start(opt_frame, False, False, 4)

    self.add(vbox)

    self.show_all()

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
    self.token_network_filter.add_filter(filter)

  # Remove the selected filter
  def remove_selected_filter(self):
    selection = self.filter_view.get_selection()
    model, iter = selection.get_selected()
    if iter:
      model.remove(iter)

  # Replace the currently selected filter
  def replace_filter(self,widget):
    selection = self.filter_view.get_selection()
    model, iter = selection.get_selected()
    first_iter = self.store.get_iter_first()
    if iter:
      self.store.remove(iter)
    elif first_iter:
      self.store.remove(first_iter)
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
    self.filter_window.connect("destroy",gtk.main_quit)
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
    self.filter_window.destroy()
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
    token_network_filter = TokenNetworkFilter(assembly,token_network)
    
    # Attach token network window to token network
    self.filter_window.set_token_network_filter(token_network_filter)

    print "Adding filter..."
    self.filter_window.filter_entry.set_text("merged")
    self.filter_window.add_but.emit("clicked")

    time.sleep(5)

if __name__ == '__main__':
  unittest.main()
