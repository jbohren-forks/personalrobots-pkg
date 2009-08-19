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
from timeline_window import ReactorPanel,TimelineWindow

from property_window import PropertyWindowFactory
from token_network import TokenNetwork
from token_network_window import TokenNetworkWindow
from token_network_filter import TokenNetworkFilter
from token_network_filter_window import TokenNetworkFilterWindow

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

  # Create timeline window
  timeline_window = TimelineWindow()
  db_reader_window.register_listener(timeline_window.set_assemblies)

  # Create token network graph generator
  token_network = TokenNetwork()
  db_reader_window.register_listener(token_network.set_assembly)

  # Create token network window
  token_network_window = TokenNetworkWindow(token_network)
  token_network_window.register_listener(PropertyWindowFactory)

  # Create token network filter window
  token_network_filter = TokenNetworkFilter(token_network)
  token_network_filter_window = TokenNetworkFilterWindow(token_network_filter)
  ReactorPanel.token_network_filter_window = token_network_filter_window

  db_reader_window.w.present()

  # Set up signal handler
  signal.signal(signal.SIGINT, signal.SIG_DFL) 

  gtk.main()

if __name__ == '__main__':
  main()
