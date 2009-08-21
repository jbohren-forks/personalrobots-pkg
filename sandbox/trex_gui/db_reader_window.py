#!/usr/bin/env python

# System modules
import sys
import os,stat

import time
import threading

import gtk
import gtk.glade

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
class DbReaderWindow():
  def __init__(self,db_reader=DbReader(),log_path="."):
    # Store db reader
    self.db_reader = db_reader

    # Store path state information
    self.log_path = log_path
    self.reactor_name = ""
    self.reactor_names = []
    self.ticks = []
    self.tick = 0
    self.status_text = ""

    # Initialize assembly dict self.db_cores[reactor_name]
    self.db_cores = {}

    # Listener structures
    self.listeners = []

    # Create glade window
    tree = gtk.glade.XML("db_reader_window.glade")
    self.w = tree.get_widget("db_reader_window")
    self.w.set_title("Assembly Navigator Control")

    # Add references to all widgets
    for w in tree.get_widget_prefix('_'):
      name = w.get_name()[1:]
      # Make sure we don't clobber existing attributes
      assert not hasattr(self, name)
      setattr(self, name, w)

    self.path_chooser.set_filename(self.log_path)

    self.path_chooser.connect("selection-changed",self.on_change_log_path)
    self.reactor_chooser.connect("changed",self.on_change_reactor)

    self.tick_combo_model = gtk.ListStore(str)
    self.tick_combo_entry.set_model(self.tick_combo_model)
    self.tick_combo_entry.set_text_column(0)

    self.tick_entry.set_text(str(self.tick))
    self.w.set_default(self.go_but)
    self.go_but.connect("clicked",self.on_tick_set)

    self.st_but.connect("clicked",self.on_tick_set_index,0)
    self.bk_but.connect("clicked",self.on_tick_inc_index,-1)
    self.fw_but.connect("clicked",self.on_tick_inc_index,1)
    self.en_but.connect("clicked",self.on_tick_set_index,-1)
    
    self.latest_toggle.connect("toggled",self.on_toggle_latest)

    # Connect the load setting controls
    self.assembly_ticks_only_check.connect("toggled",self.on_change_load_settings)
    self.load_available_assembly_check.connect("toggled",self.on_change_load_settings)

    self.assembly_ticks_only = False
    self.load_available_assemblies = True

    # Spin up the reload thread used to track the latest file
    self.do_reload = False
    self.running = True
    self.reload_lock = threading.RLock()
    self.reload_thread = threading.Thread(target=self.latest_reload_loop)
    self.reload_thread.start()

    self.w.connect("destroy", self.on_destroy)
    self.w.show()
  
  ############################################################################
  # Event Handlers
  ############################################################################
  
  # Destruction handler to clean up threads
  def on_destroy(self,widget):
    self.running = False
    self.reload_thread.join()
    return True

  # Callback for updating the log_path and gui fields when the log_path is changed
  def on_change_log_path(self,widget):
    self.log_path = self.path_chooser.get_filename()
    self.load_available_reactors()
    self.load_available_ticks()

    # Try to load the latest tick
    if len(self.ticks) > 0:
      self.load_tick(self.ticks[-1])

  # Callback for updating the reactor path when a different reactor is selected
  def on_change_reactor(self,widget):
    self.reactor_name = self.reactor_chooser.get_active_text()
    # Note there is no need to re-load ticks because these are the same across reactors
    if len(self.ticks) > 0:
      self.load_tick(self.tick)

  # Callback for setting the tick value
  def on_tick_set(self,widget):
    self.load_tick(self.tick_entry.get_text())

  # Callback for setting the tick index
  def on_tick_set_index(self,widget,index):
    self.load_tick(self.ticks[index])

  # Callback for incrementing the tick index
  def on_tick_inc_index(self,widget,index_inc):
    tick = self.tick_increment(index_inc)
    self.load_tick(tick)

  # Callback for toggling latest tick tracking
  def on_toggle_latest(self,widget):
    enabled = self.latest_toggle.get_active()
    self.reload_lock.acquire()
    self.do_reload = enabled
    self.reload_lock.release()

    self.update_available_buttons();

  # Callback for changing load parameters
  def on_change_load_settings(self,widget):
    # Get the tick restrictions
    self.assembly_ticks_only = self.assembly_ticks_only_check.get_active()
    self.load_available_assemblies = self.load_available_assembly_check.get_active()

  ############################################################################
  # GUI interaction
  ############################################################################

  # Load the reactors in the current log path and update the drop-down list accordingly
  def load_available_reactors(self):
    # Initialize available reactir names
    available_reactor_names = []

    try:
      # Get the available reactors from the db reader
      available_reactor_names = self.db_reader.get_available_reactors(self.log_path)

    except OSError:
      self.set_status("No reactor list at log path.")
    except:
      self.set_status("Failed to load reactor list from log path.")
      self.reactor_names = []
      self.reactor_name = ""

    # If any one name has been removed, refresh the list entirely
    if len(self.reactor_names) != len(available_reactor_names) or not all([name in self.reactor_names for name in available_reactor_names]):
      # Empty the dropdown list
      for name in self.reactor_names:
	self.reactor_chooser.remove_text(0)
      self.reactor_names = []

    # Append any names that are found but not already registered
    for name in [name for name in available_reactor_names if name not in self.reactor_names]:
      self.reactor_chooser.append_text(name)
      self.reactor_names.append(name)

    # Set the first option in the list to be active
    if len(self.reactor_names) > 0 and self.reactor_chooser.get_active() == -1:
      self.reactor_chooser.set_active(0)
      self.reactor_name = self.reactor_names[0]

  # Load the available ticks and update the buttons accordingly
  def load_available_ticks(self):
    # Initialize available ticks
    available_ticks = []

    try:
      if self.assembly_ticks_only:
	available_ticks = self.db_reader.get_available_assemblies(self.log_path,self.reactor_name)
      else:
	available_ticks = self.db_reader.get_available_db_cores(self.log_path,self.reactor_name)
    except:
      if self.reactor_name != "":
	self.set_status("Failed to load any ticks from log path.")
      self.tick = 0
      available_ticks = []
      self.db_cores = {}
      self.reactor_name = ""
      self.notify_listeners()

    if len(available_ticks) != len(self.ticks):
      # Append any names that are found but not already registered
      for t in [t for t in available_ticks if t not in self.ticks]:
	self.tick_combo_model.append([str(t)])
      self.ticks = available_ticks

    # Make sure the selected tick is avaiable
    if self.tick not in self.ticks and len(self.ticks) > 0:
      # Set the tick to the latest tick
      self.tick = self.ticks[-1]
      # Update the display
      self.update_tick_entry()
      # Load the assembly
      self.load_tick(self.tick)

    # Enable and disable buttons accordingly
    self.update_available_buttons()

  # This enables and disables the navigator buttons based on the available ticks
  def update_available_buttons(self):
    # Check if the latest tracking button is enabled
    latest_enabled = self.latest_toggle.get_active()

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
	  self.load_tick(self.ticks[-1])

      self.reload_lock.release()
      gtk.gdk.threads_leave()

      # Sleep before trying again
      time.sleep(0.1)

  ############################################################################
  # DbReader interaction
  ############################################################################

  # Get all available data for a given tick from the db_reader
  def load_tick(self,new_tick):
    try:
      new_tick = int(new_tick)

      # Check if this tick exists
      if new_tick not in self.ticks:
	raise IOError

      # Clear temporary dict
      db_cores = {}

      # load data for each reactor
      for reactor_name in self.reactor_names:
	# Load db cores
	db_core = self.db_reader.load_db(
	  self.log_path,
	  reactor_name,
	  new_tick)

	# Check load settings for assemblies
	if self.load_available_assemblies or self.assembly_ticks_only:
	  try:
	    assembly = self.db_reader.load_assembly(
	      self.log_path,
	      reactor_name,
	      new_tick)
	    # Append the assembly to this db_core
	    db_core.assembly = assembly
	  except:
	    pass

	# Append db core to temporary dict
	db_cores[reactor_name] = db_core

      # If we get here, the tick was loaded successfully
      self.db_cores = db_cores
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
	listener(self.db_cores, self.reactor_name)
      except:
	print "Failed to notify listener: "+str(listener)
	raise

# Testing utilities
class GtkTester():
  def spawn_gtk_thread(self):
    # Spawn the window
    gtk_thread = threading.Thread(target=self.gtk_thread)
    gtk_thread.start()

  def gtk_thread(self):
    # Spawn a thread to run gtk in
    print "Spawning gtk thread..."
    self.db_reader_window.w.connect("destroy",gtk.main_quit)
    gtk.main()

# Define a simple listener for callbacks
class SimpleAssemblyListener():
  def __init__(self):
    self.rules = {}
    self.tokens = {}

  def cb_rules(self,db_cores,reactor_name):
    print db_cores
    if reactor_name:
      self.rules = db_cores[reactor_name].assembly.rules

  def cb_tokens(self,db_cores,reactor_name):
    if reactor_name:
      self.tokens = db_cores[reactor_name].assembly.tokens

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
    self.db_reader_window.w.destroy()
    time.sleep(5)

  # Test basic user-driven reading of logs
  def test_reading(self):
    # Wait for the window to come up
    time.sleep(2)

    # Define data constants
    LOG_PATH = os.path.abspath("./test/db_reader")
    REACTORS = ["pr2.doorman","pr2.navigator","pr2.driver"]
    TICKS = range(700)
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

    print "Enabling assembly restriction..."
    self.db_reader_window.assembly_ticks_only_check.set_active(True)
    self.db_reader_window.assembly_ticks_only_check.emit("toggled")
    time.sleep(4)

    print "Checking listeners were notified..."
    self.assert_(len(self.listener.rules) > 0)
    self.assert_(len(self.listener.tokens) > 0)

    print "Disabling assembly restriction..."
    self.db_reader_window.assembly_ticks_only_check.set_active(False)
    self.db_reader_window.assembly_ticks_only_check.emit("toggled")
    time.sleep(4)

    print "Checking tick entry..."
    self.db_reader_window.tick_entry.set_text(str(GO_TICK))
    time.sleep(1)
    self.db_reader_window.go_but.emit("clicked")
    time.sleep(1)
    self.assert_(self.db_reader_window.tick == GO_TICK)

    print "Checking invalid tick entry reversion..."
    self.db_reader_window.tick_entry.set_text("100000")
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
  def _test_autoload(self):
    # Wait for the window to come up
    time.sleep(2)

    LOG_PATH = os.path.abspath("./test")
    REACTOR_NAME = "NOT_A_REAL_REACTOR"
    REACTOR_ASSEMBLY_PATH = os.path.abspath("./test/db_reader_window/%s" % DbReader.ASSEMBLY_PATH)
    REACTOR_DB_PATH = os.path.abspath("./test/db_reader_window/%s" % DbReader.DB_PATH)
    REACTOR_PATH = os.path.join(REACTOR_ASSEMBLY_PATH, REACTOR_NAME)
    REACTOR_TICK_PATH = os.path.join(REACTOR_PATH,"tick100")
    REACTOR_TICK_PATH_2 = os.path.join(REACTOR_PATH,"tick200")

    print "Setting the log path..."
    self.db_reader_window.path_chooser.set_filename(LOG_PATH)
    time.sleep(2)

    print "Making new reactor directory..."
    try:
      os.rmdir(REACTOR_TICK_PATH_2)
      os.rmdir(REACTOR_TICK_PATH)
      os.rmdir(REACTOR_PATH)
      os.rmdir(REACTOR_ASSEMBLY_PATH)
    except:
      pass
    os.makedirs(REACTOR_PATH)
    time.sleep(4)

    print "Checking for automatic reactor pick-up..."
    self.assert_([REACTOR_NAME] == self.db_reader_window.reactor_names)

    print "Making new tick directory"
    os.makedirs(REACTOR_TICK_PATH)
    time.sleep(4)

    print "Checking for automatic tick pick-up..."
    self.assert_([100] == self.db_reader_window.ticks)

    print "Enabling automatic tick tracking..."
    self.db_reader_window.latest_toggle.set_active(True)
    self.db_reader_window.latest_toggle.emit("toggled")
    os.makedirs(REACTOR_TICK_PATH_2)
    time.sleep(4)

    print "Checking for automatic tick tracking..."
    self.assert_(self.db_reader_window.status_text == "Could not load Tick [%d] from \"%s\"!" % (200,REACTOR_NAME))

    print "Removing new reactor directory..."
    os.rmdir(REACTOR_TICK_PATH_2)
    os.rmdir(REACTOR_TICK_PATH)
    os.rmdir(REACTOR_PATH)
    os.rmdir(REACTOR_ASSEMBLY_PATH)
    time.sleep(4)

    print "Checking for no reactor pick-up..."
    self.assert_(len(self.db_reader_window.reactor_names) == 0)

if __name__ == '__main__':
  unittest.main()
