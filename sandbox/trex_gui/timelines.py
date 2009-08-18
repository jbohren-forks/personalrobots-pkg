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
  db_reader_window.register_listener(timeline_window.set_assemblies)

  db_reader_window.w.present()

  # Set up signal handler
  signal.signal(signal.SIGINT, signal.SIG_DFL) 

  gtk.main()

if __name__ == '__main__':
  main()
