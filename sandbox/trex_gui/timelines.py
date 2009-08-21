#!/usr/bin/env python

# System modules
import sys,os
import unittest
import signal
import threading, time
import gtk

# TREX modules
from assembly import Assembly,Entity,Rule,Token,Slot,Variable
from db_reader import DbReader
from db_reader_window import DbReaderWindow
from timeline_window import TimelineWindow

from token_network import TokenNetwork
from token_network_window import TokenNetworkWindow
from token_network_filter import TokenNetworkFilter
from token_network_filter_window import TokenNetworkFilterWindow
from property_window import PropertyWindowFactory

def main():
  # Process cli arguments
  if len(sys.argv) > 1:
    log_path = sys.argv[1]
  else:
    log_path = "."

  # Initialize gtk multithread support
  gtk.gdk.threads_init()

  # Create db reader window
  db_reader_window = DbReaderWindow(log_path=log_path)
  db_reader_window.w.connect("destroy",gtk.main_quit)

  # Create token network graph generator
  timeline_window = TimelineWindow()
  db_reader_window.register_listener(timeline_window.set_db_cores)

  ############################################################

  # Create token network graph generator
  token_network = TokenNetwork()

  # Create token network window
  token_network_window = TokenNetworkWindow(token_network)
  token_network_window.register_listener(PropertyWindowFactory)

  # Create token network filter window
  token_network_filter = TokenNetworkFilter(token_network)
  token_network_filter_window = TokenNetworkFilterWindow(token_network_filter)

  def HilightInTokenNetwork(db_cores,token):
    # Set assembly in token network
    token_network.set_db_cores({db_core.reactor_name : db_core},db_core.reactor_name)
    
    # Set the filter for the token key
    token_network_filter_window.filter_entry.set_text(str(token.key))
    token_network_filter_window.rep_but.emit("clicked")

  # Register context extensions
  timeline_window.register_context_extension("View token properties...",PropertyWindowFactory)
  timeline_window.register_context_extension("Hilight in token network...",HilightInTokenNetwork)

  # Bring the db reader forward
  db_reader_window.w.present()

  # Set up signal handler
  signal.signal(signal.SIGINT, signal.SIG_DFL) 

  gtk.main()

if __name__ == '__main__':
  main()
