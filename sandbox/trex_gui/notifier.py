#!/usr/bin/env python

class Notifier():
  def __init__(self):
    # Initialize listener list
    self.listeners = []

    # Initialize the notifying flag
    # This is set during notification, to prevent infinite callback chaining
    self.notifying = False
    self.notify_pending = 0

  ############################################################################
  # Callback registration for classes that display loaded data
  # These callbacks are called with a reference to this token network
  ############################################################################

  # Function to register a listener callback on network update
  def register_listener(self,listener_cb):
    self.listeners.append(listener_cb)
  
  # Function to unregister a listener callback
  def unregister_listener(self,listener_cb):
    if listener_cb in self.listeners:
      self.listeners.remove(listener_cb)

  # Function that notifies listeners when the network has been modified in some way
  def notify_listeners(self):
    # Increment the number of pending notifications
    self.notify_pending = self.notify_pending + 1
    if not self.notifying:
      # Raise notifying flag
      self.notifying = True
      while self.notify_pending > 0:
	# Notify listeners
	for listener in self.listeners:
	  try:
	    #print "LISTENER HOLLER BACK: " + str(listener)
	    listener(self)
	  except:
	    print "Failed to notify listener: "+str(listener)
	    raise
	# Decrement notify_pending
	self.notify_pending = self.notify_pending - 1
      # Lower notifying flag
      self.notifying = False
