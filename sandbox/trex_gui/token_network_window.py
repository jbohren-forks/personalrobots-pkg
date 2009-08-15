#!/usr/bin/env python

# System modules
import sys
import os,stat

import time
import threading

import gtk

import unittest

# 3rd Party modules
sys.path.insert(0, os.path.abspath("./ext"))
import xdot

# TREX modules
from token_network import TokenNetwork

##############################################################################
# TokenNetworkWindow
#   This window allows the user to browse and interact with the token network.
##############################################################################

class TokenNetworkWindow(xdot.DotWindow):
  def __init__(self,token_network=TokenNetwork()):
    xdot.DotWindow.__init__(self)

    # Initialize listener list
    self.listeners = []

    # Initialize token network
    self.token_network = token_network
    self.token_network.register_listener(self.set_token_network)

    # Initialize gui stuff
    self.set_title("Token Network")
    self.widget.connect('clicked', self.on_node_clicked)
    self.set_size_request(1500,500)
    self.set_position(gtk.WIN_POS_CENTER)

  ############################################################################
  # Functions for setting the graph dotcode
  ############################################################################

  # Method to push a token network into the viewer
  def set_token_network(self,token_network):
    try:
      # Store a reference to the token network
      self.token_network = token_network
      # Set the dotcode
      self.set_dotcode(token_network.get_dotcode())
    except ZeroDivisionError:
      pass
    except:
      raise

  ############################################################################
  # Callback registration for classes that process loaded data
  ############################################################################

  # Function to register a listener callback
  def register_listener(self,listener_cb):
    self.listeners.append(listener_cb)
  
  # Function to unregister a listener callback
  def unregister_listener(self,listener_cb):
    if listener_cb in self.listeners:
      self.listeners.remove(listener_cb)

  # Notify listeners when a node is clicked
  def on_node_clicked(self, widget, userdata, event):
    for listener in self.listeners:
      try:
	listener(self.token_network.assembly,self.token_network.deserialize(userdata))
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
    self.token_network_window.connect("destroy",gtk.main_quit)
    gtk.main()

# Define a simple listener for callbacks
class SimpleClickListener():
  def __init__(self):
    self.rules = {}
    self.tokens = {}

  def cb_clicked(self,):
    self.rules = assembly.rules

# Unit tests
class TestTokenNetworkWindow(unittest.TestCase,GtkTester):
  # Create the gtk thread and window structure
  def setUp(self):
    # Initialize GTK Python threading functionality
    gtk.gdk.threads_init()
    # Create a new db reader window
    self.token_network_window = TokenNetworkWindow()

    # Create a new listener
    self.listener = SimpleClickListener()
    # Register callbacks
    self.token_network_window.register_listener(self.listener.cb_clicked)

    # Spawn the window
    self.spawn_gtk_thread()
    
  # Destroy window and kill gtk
  def tearDown(self):
    print "Killing The window..."
    self.token_network_window.destroy()
    time.sleep(5)

  # Test basic display of token networks
  def test_push_network(self):
    from assembly import construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create token network
    token_network = TokenNetwork()
    token_network.set_assembly(assembly)
    
    # Attach token network window to token network
    self.token_network_window.set_token_network(token_network)

    time.sleep(5)

  # Test the auto-redraw of a network
  def test_cb_network(self):
    from assembly import Assembly,construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create token network
    token_network = TokenNetwork()
    time.sleep(1)
    
    # Attach token network window to token network
    token_network.register_listener(self.token_network_window.set_token_network)
    time.sleep(1)

    print "Updating assembly in token network..."
    token_network.set_assembly(assembly)
    time.sleep(1)

    print "Changing a hilighted token..."
    tok = assembly.tokens.values()[0]
    token_network.hilight(tok,True)
    token_network.notify_listeners()
    time.sleep(1)

    print "Clearing assembly in token network..."
    token_network.set_assembly(Assembly())
    time.sleep(1)

if __name__ == '__main__':
  unittest.main()
