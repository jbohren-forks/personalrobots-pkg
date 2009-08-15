#!/usr/bin/env python

# System modules
import sys
import os,stat

import time
import threading

import gtk

import unittest

# TREX modules
from db_reader import DbReader
from assembly import Assembly

##############################################################################
# DbReaderWindow
#   This window provides a user with controls to step to any tick and re-run
#   the data loader. It also does continuous polling of the file system in
#   order to grab data for visualization as it is written to disk.
##############################################################################

# DbReaderWindow GTK widget class
class DbReaderWindow(gtk.Window):
  def __init__(self,db_reader=DbReader(),log_path="."):
    super(DbReaderWindow, self).__init__()
    self.set_title("Assembly Navigator Control")
    self.set_position(gtk.WIN_POS_CENTER)

    # Store db reader
    self.db_reader = db_reader

    # Store path state information
    self.log_path = log_path
    self.reactor_name = ""
    self.reactor_names = []
    self.ticks = []
    self.tick = 0
    self.status_text = ""
    self.assembly = Assembly()

    # Listener structures
    self.listeners = []

    vbox = gtk.VBox(False, 8)

    ################################################

    self.statusbar = gtk.Statusbar()
    vbox.pack_end(self.statusbar, False, False, 0)

    ################################################
    mb = gtk.MenuBar()

    file_menu = gtk.Menu()
    file_menu_item = gtk.MenuItem("File")
    file_menu_item.set_submenu(file_menu)

    exit = gtk.MenuItem("Exit")
    exit.connect("activate",quit)
    file_menu.append(exit)

    mb.append(file_menu_item)

    vbox.pack_start(mb,False,False,0)

    ################################################
    source_frame = gtk.Frame("Source: ")
    source_table = gtk.Table(2,2,homogeneous=False)


    self.fc_dialog = gtk.FileChooserDialog(
	"Select a TREX log path...",
	action=gtk.FILE_CHOOSER_ACTION_OPEN | gtk.FILE_CHOOSER_ACTION_SELECT_FOLDER,
	buttons=(gtk.STOCK_CANCEL,gtk.RESPONSE_CANCEL,gtk.STOCK_OPEN,gtk.RESPONSE_OK))
    self.fc_dialog.set_default_response(gtk.RESPONSE_OK)
    self.fc_dialog.set_filename(self.log_path)

    self.path_chooser = gtk.FileChooserButton(self.fc_dialog)
    self.path_chooser.connect("selection-changed",self.on_change_log_path)
    path_label = gtk.Label("Path: ")

    halign = gtk.Alignment(1,0.5,0,0.5)
    halign.add(path_label)

    source_table.attach(halign, 0,1,0,1, gtk.FILL, gtk.FILL,0,0)
    source_table.attach(self.path_chooser, 1,2,0,1, gtk.FILL|gtk.EXPAND, gtk.SHRINK)

    self.reactor_chooser = gtk.combo_box_new_text()
    self.reactor_chooser.connect("changed",self.on_change_reactor)

    reactor_label = gtk.Label("Reactor: ")

    source_table.attach(reactor_label, 0,1,1,2, gtk.SHRINK, gtk.SHRINK)
    source_table.attach(self.reactor_chooser, 1,2,1,2, gtk.FILL|gtk.EXPAND, gtk.SHRINK)

    padding = gtk.Alignment(1.0,0.5,1,1)
    padding.set_padding(4,4,4,4)
    padding.add(source_table)

    source_frame.add(padding)
    vbox.pack_start(source_frame, False, False, 4)

    ################################################

    tick_frame = gtk.Frame("Tick Control")

    self.tick_entry = gtk.Entry()
    self.tick_entry.set_activates_default(True)
    self.tick_entry.set_text(str(self.tick))
    self.go_but = gtk.Button("go:")
    self.go_but.set_flags(gtk.CAN_DEFAULT)
    self.set_default(self.go_but)
    self.go_but.connect("clicked",self.on_tick_set)

    self.en_but = gtk.Button(">|")
    self.fw_but = gtk.Button(">")
    self.bk_but = gtk.Button("<")
    self.st_but = gtk.Button("|<")

    self.latest_but = gtk.ToggleButton("Latest")

    self.st_but.connect("clicked",self.on_tick_set_index,0)
    self.bk_but.connect("clicked",self.on_tick_inc_index,-1)
    self.fw_but.connect("clicked",self.on_tick_inc_index,1)
    self.en_but.connect("clicked",self.on_tick_set_index,-1)
    
    self.latest_but.connect("toggled",self.on_toggle_latest)

    nav_hbox = gtk.HBox(False,4)

    nav_hbox.pack_end(self.latest_but, False, False, 0)
    nav_hbox.pack_end(self.en_but, False, False, 0)
    nav_hbox.pack_end(self.fw_but, False, False, 0)
    nav_hbox.pack_end(self.tick_entry, True, True, 0)
    nav_hbox.pack_end(self.go_but, False, False, 0)
    nav_hbox.pack_end(self.bk_but, False, False, 0)
    nav_hbox.pack_end(self.st_but, False, False, 0)

    padding = gtk.Alignment(1.0,0.5,1,1)
    padding.set_padding(4,4,4,4)
    padding.add(nav_hbox)

    tick_frame.add(padding)
    vbox.pack_start(tick_frame, False, False, 4)
    
    self.add(vbox)

    # Spin up the reload thread used to track the latest file
    self.do_reload = False
    self.running = True
    self.reload_lock = threading.RLock()
    self.reload_thread = threading.Thread(target=self.latest_reload_loop)
    self.reload_thread.start()

    self.connect("destroy", self.on_destroy)
    self.show_all()
  
  ############################################################################
  # Event Handlers
  ############################################################################
  
  # Destruction handler to clean up threads
  def on_destroy(self,widget):
    self.running = False
    self.reload_thread.join()

  # Callback for updating the log_path and gui fields when the log_path is changed
  def on_change_log_path(self,widget):
    self.log_path = self.path_chooser.get_filename()
    self.load_available_reactors()
    self.load_available_ticks()

    # Try to load the latest tick
    if len(self.ticks) > 0:
      self.load_assembly(self.ticks[-1])

  # Callback for updating the reactor path when a different reactor is selected
  def on_change_reactor(self,widget):
    self.reactor_name = self.reactor_chooser.get_active_text()
    # Note there is no need to re-load ticks because these are the same across reactors
    if len(self.ticks) > 0:
      self.load_assembly(self.tick)

  # Callback for setting the tick value
  def on_tick_set(self,widget):
    self.load_assembly(self.tick_entry.get_text())

  # Callback for setting the tick index
  def on_tick_set_index(self,widget,index):
    self.load_assembly(self.ticks[index])

  # Callback for incrementing the tick index
  def on_tick_inc_index(self,widget,index_inc):
    tick = self.tick_increment(index_inc)
    self.load_assembly(tick)

  # Callback for toggling latest tick tracking
  def on_toggle_latest(self,widget):
    enabled = self.latest_but.get_active()
    self.reload_lock.acquire()
    self.do_reload = enabled
    self.reload_lock.release()

    self.update_available_buttons();

  ############################################################################
  # GUI interaction
  ############################################################################

  # Load the reactors in the current log path and update the drop-down list accordingly
  def load_available_reactors(self):
    try:
      # Get the available reactors from the db reader
      available_reactor_names = self.db_reader.get_available_reactors(self.log_path)

      # If any one name has been removed, refresh the list entirely
      if not all([name in self.reactor_names for name in available_reactor_names]):
	# Empty the dropdown list
	for name in self.reactor_names:
	  self.reactor_chooser.remove_text(0)
	self.reactor_names = []

      # Append any names that are found but not already registered
      for name in [name for name in available_reactor_names if name not in self.reactor_names]:
	self.reactor_chooser.append_text(name)
	self.reactor_names.append(name)

      # Set the first option in the list to be active
      if self.reactor_chooser.get_active() == -1:
	self.reactor_chooser.set_active(0)
    except OSError:
      self.set_status("No reactor list at log path.")
    except:
      self.set_status("Failed to load reactor list from log path.")
      self.reactor_names = []
      self.reactor_name = ""
      raise

  # Load the available ticks and update the buttons accordingly
  def load_available_ticks(self):
    try:
      self.ticks = self.db_reader.get_available_ticks(self.log_path,self.reactor_name)
    except:
      if self.reactor_name != "":
	self.set_status("Failed to load any ticks from log path.")
      self.tick = 0
      self.ticks = []

    # Make sure the selected tick is avaiable
    if self.tick not in self.ticks and len(self.ticks) > 0:
      # Set the tick to the latest tick
      self.tick = self.ticks[-1]
      # Update the display
      self.update_tick_entry()
      # Load the assembly
      self.load_assembly(self.tick)

    # Enable and disable buttons accordingly
    self.update_available_buttons()

  # This enables and disables the navigator buttons based on the available ticks
  def update_available_buttons(self):
    # Check if the latest tracking button is enabled
    latest_enabled = self.latest_but.get_active()

    # If there are no reactors, disable the combobox
    self.reactor_chooser.set_sensitive(len(self.reactor_names) > 0)

    # Set the button sensitivities
    if len(self.ticks) == 0 or latest_enabled:
      # Disable the other tick control buttons
      self.tick_entry.set_sensitive(False)
      self.go_but.set_sensitive(False)
      self.bk_but.set_sensitive(False)
      self.st_but.set_sensitive(False)
      self.en_but.set_sensitive(False) 
      self.fw_but.set_sensitive(False)
    else:
      self.tick_entry.set_sensitive(True)
      self.go_but.set_sensitive(True)

      if self.ticks.index(self.tick) == 0:
	self.bk_but.set_sensitive(False)
	self.st_but.set_sensitive(False)
      else:
	self.bk_but.set_sensitive(True)
	self.st_but.set_sensitive(True)

      if self.ticks.index(self.tick) == len(self.ticks)-1:
	self.en_but.set_sensitive(False) 
	self.fw_but.set_sensitive(False)
      else:
	self.en_but.set_sensitive(True) 
	self.fw_but.set_sensitive(True)

  # Update the text entry to reflect the current tick
  def update_tick_entry(self):
    self.tick_entry.set_text(str(self.tick))

  # Set the status text
  def set_status(self,text):
    self.status_text = text
    self.statusbar.pop(0)
    self.statusbar.push(0,self.status_text)

  ############################################################################
  # Tick manipulation
  ############################################################################

  # Calculate a new tick by incrementing current tick some indices forward or backward
  def tick_increment(self, index_inc):
    new_tick = 0
    try:
      cur_index = self.ticks.index(self.tick)
      new_tick = self.ticks[cur_index + index_inc]
    except:
      self.set_status("Tick %d not available." % (new_tick))
    return new_tick

  ############################################################################
  # Multi-threaded polling functions
  ############################################################################

  # This function continuously reloads the available ticks in a given reactor path
  def latest_reload_loop(self):
    while self.running:
      gtk.gdk.threads_enter()
      self.reload_lock.acquire()

      # Check the available reactors
      self.load_available_reactors()

      # Check the available ticks
      self.load_available_ticks()

      # Reload if there is a newer tick available
      if self.do_reload:
	if len(self.ticks) > 0 and self.ticks[-1] != self.tick:
	  self.load_assembly(self.ticks[-1])

      self.reload_lock.release()
      gtk.gdk.threads_leave()

      # Sleep before trying again
      time.sleep(0.5)

  ############################################################################
  # DbReader interaction
  ############################################################################

  # Get an assembly from the db reader
  def load_assembly(self,new_tick):
    try:
      new_tick = int(new_tick)

      # Check if this tick exists
      if new_tick not in self.ticks:
	raise IOError

      # Load assembly
      assembly = self.db_reader.load_assembly(
	  self.log_path,
	  self.reactor_name,
	  new_tick)

      # If we get here, the tick was loaded successfully
      self.assembly = assembly
      self.tick = new_tick

      # Post an update to the status bar
      self.set_status("Loaded Tick [%d] from \"%s\"" % (self.tick,self.reactor_chooser.get_active_text()))
    except ValueError:
      self.set_status("Invalid tick entry!")
    except:
      self.set_status("Could not load Tick [%s] from \"%s\"!" % (str(new_tick),self.reactor_chooser.get_active_text()))

    # Update gui
    self.update_tick_entry()
    self.update_available_buttons()

    # Notify listeners
    self.notify_listeners()


  ############################################################################
  # Callback registration for classes that process loaded data
  ############################################################################

  # Function to register on load
  def register_listener(self,listener_cb):
    self.listeners.append(listener_cb)
  
  # Function to unregister a listener callback
  def unregister_listener(self,listener_cb):
    if listener_cb in self.listeners:
      self.listeners.remove(listener_cb)

  # Function that notifies listeners with new assembly when it is loaded
  def notify_listeners(self):
    for listener in self.listeners:
      try:
	listener(self.assembly)
      except:
	print "Failed to notify listener: "+str(listener)

# Testing utilities
class GtkTester():
  def spawn_gtk_thread(self):
    # Spawn the window
    gtk_thread = threading.Thread(target=self.gtk_thread)
    gtk_thread.start()

  def gtk_thread(self):
    # Spawn a thread to run gtk in
    print "Spawning gtk thread..."
    self.db_reader_window.connect("destroy",gtk.main_quit)
    gtk.main()

# Define a simple listener for callbacks
class SimpleAssemblyListener():
  def __init__(self):
    self.rules = {}
    self.tokens = {}

  def cb_rules(self,assembly):
    self.rules = assembly.rules

  def cb_tokens(self,assembly):
    self.tokens = assembly.tokens

# Unit tests
class TestDbReaderWindow(unittest.TestCase,GtkTester):
  # Create the gtk thread and window structure
  def setUp(self):
    # Initialize GTK Python threading functionality
    gtk.gdk.threads_init()
    # Create a new db reader
    self.db_reader = DbReader()
    # Create a new db reader window
    self.db_reader_window = DbReaderWindow(self.db_reader)

    # Create a new listener
    self.listener = SimpleAssemblyListener()
    # Register callbacks
    self.db_reader_window.register_listener(self.listener.cb_rules)
    self.db_reader_window.register_listener(self.listener.cb_tokens)

    # Spawn the window
    self.spawn_gtk_thread()
    
  # Destroy window and kill gtk
  def tearDown(self):
    print "Killing The window..."
    self.db_reader_window.destroy()
    time.sleep(5)

  # Test basic user-driven reading of logs
  def test_reading(self):
    # Wait for the window to come up
    time.sleep(2)

    # Define data constants
    LOG_PATH = os.path.abspath("./test/db_reader")
    REACTORS = ["pr2.doorman", "pr2.navigator"]
    TICKS = [100, 200, 300, 400, 500, 600, 700, 800, 900]
    GO_TICK = 500
    
    print "Setting the log path..."
    self.db_reader_window.path_chooser.set_filename(LOG_PATH)
    time.sleep(2)
    
    print "Checking available data was loaded properly..."
    # Assert the log path was loaded properly
    self.assert_(LOG_PATH == self.db_reader_window.log_path)
    # Assert the reactors were loaded properly
    self.assert_(REACTORS == self.db_reader_window.reactor_names)
    # Assert all the expected ticks were loaded properly
    print self.db_reader_window.ticks
    self.assert_(TICKS == self.db_reader_window.ticks)

    print "Checking navigation buttons..."
    self.db_reader_window.st_but.emit("clicked")
    time.sleep(1)
    self.assert_(self.db_reader_window.tick == TICKS[0])
    self.db_reader_window.en_but.emit("clicked")
    time.sleep(1)
    self.assert_(self.db_reader_window.tick == TICKS[-1])
    self.db_reader_window.bk_but.emit("clicked")
    self.assert_(self.db_reader_window.tick == TICKS[-2])
    time.sleep(1)
    self.db_reader_window.fw_but.emit("clicked")
    self.assert_(self.db_reader_window.tick == TICKS[-1])

    print "Checking listeners were notified..."
    self.assert_(len(self.listener.rules) > 0)
    self.assert_(len(self.listener.tokens) > 0)

    print "Checking tick entry..."
    self.db_reader_window.tick_entry.set_text(str(GO_TICK))
    time.sleep(1)
    self.db_reader_window.go_but.emit("clicked")
    time.sleep(1)
    self.assert_(self.db_reader_window.tick == GO_TICK)

    print "Checking invalid tick entry reversion..."
    self.db_reader_window.tick_entry.set_text("0")
    time.sleep(1)
    self.db_reader_window.go_but.emit("clicked")
    time.sleep(1)
    self.assert_(self.db_reader_window.tick == GO_TICK)

    print "Moving to an empty path..."
    self.db_reader_window.path_chooser.set_filename(".")
    time.sleep(2)

    print "Checking listeners were notified..."
    self.assert_(len(self.listener.rules) == 0)
    self.assert_(len(self.listener.tokens) == 0)
    time.sleep(2)

  # Test auto-reload functionality
  def test_autoload(self):
    # Wait for the window to come up
    time.sleep(2)

    LOG_PATH = os.path.abspath("./test")
    DYNAMIC_REACTOR_NAME = "NOT_A_REAL_REACTOR"
    DYNAMIC_REACTOR_ASSEMBLY_PATH = os.path.abspath("./test/assembly_dumps")
    DYNAMIC_REACTOR_PATH = os.path.join(DYNAMIC_REACTOR_ASSEMBLY_PATH, DYNAMIC_REACTOR_NAME)
    DYNAMIC_REACTOR_TICK_PATH = os.path.join(DYNAMIC_REACTOR_PATH,"tick100")
    DYNAMIC_REACTOR_TICK_PATH_2 = os.path.join(DYNAMIC_REACTOR_PATH,"tick200")

    print "Setting the log path..."
    self.db_reader_window.path_chooser.set_filename(LOG_PATH)
    time.sleep(2)

    print "Making new reactor directory..."
    try:
      os.rmdir(DYNAMIC_REACTOR_TICK_PATH_2)
      os.rmdir(DYNAMIC_REACTOR_TICK_PATH)
      os.rmdir(DYNAMIC_REACTOR_PATH)
      os.rmdir(DYNAMIC_REACTOR_ASSEMBLY_PATH)
    except:
      pass
    os.makedirs(DYNAMIC_REACTOR_PATH)
    time.sleep(4)

    print "Checking for automatic reactor pick-up..."
    self.assert_([DYNAMIC_REACTOR_NAME] == self.db_reader_window.reactor_names)

    print "Making new tick directory"
    os.makedirs(DYNAMIC_REACTOR_TICK_PATH)
    time.sleep(4)

    print "Checking for automatic tick pick-up..."
    self.assert_([100] == self.db_reader_window.ticks)

    print "Enabling automatic tick tracking..."
    self.db_reader_window.latest_but.set_active(True)
    self.db_reader_window.latest_but.emit("toggled")
    os.makedirs(DYNAMIC_REACTOR_TICK_PATH_2)
    time.sleep(4)

    print "Checking for automatic tick tracking..."
    self.assert_(self.db_reader_window.status_text == "Could not load Tick [%d] from \"%s\"!" % (200,DYNAMIC_REACTOR_NAME))

    print "Removing new reactor directory..."
    os.rmdir(DYNAMIC_REACTOR_TICK_PATH_2)
    os.rmdir(DYNAMIC_REACTOR_TICK_PATH)
    os.rmdir(DYNAMIC_REACTOR_PATH)
    os.rmdir(DYNAMIC_REACTOR_ASSEMBLY_PATH)
    time.sleep(4)

    print "Checking for no reactor pick-up..."
    self.assert_(len(self.db_reader_window.reactor_names) == 0)

if __name__ == '__main__':
  unittest.main()
