#!/usr/bin/env python

# System modules
import sys
import os

import time
import threading

import gtk

import unittest
import traceback

# TREX modules
from db_reader import DbReader

##############################################################################
# DbViewerWindow
#   This window provides a user with controls to step to any tick and re-run
#   the data loader.
##############################################################################

class DbReaderWindow(gtk.Window):
  def __init__(self,db_reader,log_path="."):
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
    self.assembly = None

    # Spin up the reload thread used to track the latest file
    self.do_reload = False
    self.running = True
    self.reload_lock = threading.RLock()
    self.reload_thread = threading.Thread(target=self.latest_reload_loop)
    self.reload_thread.start()

    vbox = gtk.VBox(False, 8)
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

    search_hbox = gtk.HBox(False,4)

    search_hbox.pack_end(self.latest_but, False, False, 0)
    search_hbox.pack_end(self.en_but, False, False, 0)
    search_hbox.pack_end(self.fw_but, False, False, 0)
    search_hbox.pack_end(self.tick_entry, True, True, 0)
    search_hbox.pack_end(self.go_but, False, False, 0)
    search_hbox.pack_end(self.bk_but, False, False, 0)
    search_hbox.pack_end(self.st_but, False, False, 0)

    padding = gtk.Alignment(1.0,0.5,1,1)
    padding.set_padding(4,4,4,4)
    padding.add(search_hbox)

    tick_frame.add(padding)
    vbox.pack_start(tick_frame, False, False, 4)
    
    ################################################

    self.statusbar = gtk.Statusbar()
    vbox.pack_end(self.statusbar, False, False, 0)

    self.add(vbox)

    # try to load the latest
    if len(self.ticks) > 0:
      self.load_assembly(self.ticks[-1])

    self.connect("destroy", self.on_destroy)
    self.show_all()
  
  ############################################################################
  # Event Handlers
  ############################################################################
  
  def on_destroy(self,widget):
    self.running = False
    self.reload_thread.join()

  # Callback for updating the log_path when it is changed
  # This also re-loads the available reactors and ticks
  def on_change_log_path(self,widget):
    self.log_path = self.path_chooser.get_filename()
    self.load_available_reactors()
    self.load_available_ticks()

  # Callback for updating the reactor path when a different reactor is selected
  def on_change_reactor(self,widget):
    self.reactor_name = self.reactor_chooser.get_active_text()
    # Note there is no need to re-load ticks because these are the same across reactors

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
  # Multi-threaded polling functions
  ############################################################################

  # This function continuously reloads the available ticks in a given reactor path
  def latest_reload_loop(self):
    while self.running:
      gtk.gdk.threads_enter()
      self.reload_lock.acquire()
      enabled = self.do_reload
      if self.do_reload:
	# Check the available ticks
	self.load_available_ticks()
	# Re-load if there is a newer tick available
	if len(self.ticks) > 0 and self.ticks[-1] != self.tick:
	  self.load_assembly(self.ticks[-1])
      self.reload_lock.release()
      gtk.gdk.threads_leave()
      # Sleep before trying again
      time.sleep(1.0)

  ############################################################################
  # DbReader interaction
  ############################################################################

  def update_fields(self):
    # Load the available reactors and ticks based on an updated log path
    self.load_available_reactors()
    self.load_available_ticks()

  # Load the reactors in the current log path and update the drop-down list accordingly
  def load_available_reactors(self):
    try:
      # Empty the dropdown list
      for p in self.reactor_names:
	self.reactor_chooser.remove_text(0)

      # Get the available reactors from the db reader
      self.reactor_names = self.db_reader.get_available_reactors(self.log_path)

      # Add the reactors to the dropdown list
      for reactor_name in self.reactor_names:
	self.reactor_chooser.append_text(reactor_name)

      # Set the first option in the list to be active
      self.reactor_chooser.set_active(0)
    except:
      self.statusbar.push(0,"Failed to load reactor list from log path.")

  # Load the available ticks and update the buttons accordingly
  def load_available_ticks(self):
    try:
      self.ticks = self.db_reader.get_available_ticks(self.log_path,self.reactor_name)
      # Make sure the selected tick is avaiable
      if self.tick not in self.ticks and len(self.ticks) > 0:
	# Set the tick to the latest tick
	self.tick = self.ticks[-1]
	# Update the display
	self.update_tick_entry()
	# Load the assembly
	self.load_assembly(self.tick)
    except:
      self.statusbar.push(0,"Failed to load any ticks from log path.")

    # Enable and disable buttons accordingly
    self.update_available_buttons()

  # Calculate a new tick by incrementing current tick some indices forward or backward
  def tick_increment(self, index_inc):
    new_tick = 0
    try:
      cur_index = self.ticks.index(self.tick)
      new_tick = self.ticks[cur_index + index_inc]
    except:
      self.statusbar.push(0,"Tick %d not available." % (new_tick))
    return new_tick

  # This enables and disables the navigator buttons based on the available ticks
  def update_available_buttons(self):
    # Check if the latest tracking button is enabled
    latest_enabled = self.latest_but.get_active()

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

  # Get an assembly from the db reader
  def load_assembly(self,new_tick):
    try:
      new_tick = int(new_tick)

      # Check if this tick exists
      if new_tick not in self.ticks:
	self.statusbar.push(0,"Tick %d not available." % (new_tick))
	return

      # Load assembly
      self.assembly = self.db_reader.load_assembly(
	  self.log_path,
	  self.reactor_name,
	  self.tick)

      # If we get here, the tick was loaded successfully
      self.tick = new_tick
      self.update_tick_entry()
      self.update_available_buttons()

      # Post an update to the status bar
      self.statusbar.push(0,"Loaded Tick [%d] from \"%s\"" % (self.tick,self.reactor_chooser.get_active_text()))
    except ValueError:
      self.statusbar.push(0,"Invalid tick entry!")
    except:
      self.statusbar.push(0,"Could not load Tick [%d] from \"%s\"!" % (self.tick,self.reactor_chooser.get_active_text()))


# Testing code
class TestDbReaderWindow(unittest.TestCase):
  def setUp(self):
    self.db_reader_window = None

  def spawn_gtk_thread(self):
    # Spawn the window
    gtk.gdk.threads_init()
    gtk_thread = threading.Thread(target=self.gtk_thread)
    gtk_thread.start()

  def gtk_thread(self):
    # Create a new db reader
    self.db_reader = DbReader()
    # Create a new db reader window
    self.db_reader_window = DbReaderWindow(self.db_reader)
    self.db_reader_window.connect("destroy",gtk.main_quit)
    # Spawn a thread to run gtk in
    print "Spawning gtk thread..."
    gtk.main()

  def test_display(self):
    # Spawn the window
    self.spawn_gtk_thread()
    # Wait two seconds
    time.sleep(2)
    # kill the window
    self.db_reader_window.destroy()

  def test_reading(self):
    # Spawn the window
    self.spawn_gtk_thread()
    
    # Wait for the window to come up
    time.sleep(2)

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
    time.sleep(1)
    self.db_reader_window.fw_but.emit("clicked")
    self.assert_(self.db_reader_window.tick == TICKS[-1])

    print "Checking tick entry..."
    self.db_reader_window.tick_entry.set_text(str(GO_TICK))
    time.sleep(1)
    self.db_reader_window.go_but.emit("clicked")
    time.sleep(1)

    print "Waiting..."
    time.sleep(5)
    # kill the window
    self.db_reader_window.destroy()
    

if __name__ == '__main__':
  unittest.main()
