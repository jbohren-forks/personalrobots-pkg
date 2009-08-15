#!/usr/bin/env python

# System modules
import sys,os
import unittest

# TREX modules
from notifier import Notifier
from assembly import Assembly,Entity,Rule,Token,Slot,Variable
from token_network import TokenNetwork

##############################################################################
# TokenNetworkFilter
#   This class provides a mechanism for filtering (setting hilights on) a
#   token network. It does not store a token network, but only reacts to
#   network updates.
#
#   The network_listener callback is provided to a TokenNetwork's listener
#   list so that any data updates can be handled accordingly.
#
#   The TokenNetworkFilter currently only supports successive string-matching
#   filters. This means that a node will be hilighted if each filter matches
#   one of it's properties (name, key, variable name, variable value, etc.)
##############################################################################

class TokenNetworkFilter():
  def __init__(self,token_network=TokenNetwork()):
    # Initialize filter list
    self.filters = []

    # Initialize assembly
    self.assembly = Assembly()

    # Initialize token network
    self.token_network = token_network
    self.token_network.register_listener(self.network_listener)

  # Add a filter
  def add_filter(self,filter):
    self.filters.append(filter)
    # Refilter the network
    self.filter_network()

  # Remove a filter by value
  def rem_filter(self,filter):
    if filter in self.filters:
      self.filters.remove(filter)
      # Refilter the network
      self.filter_network()

  # Callback for when a network gets updated
  def network_listener(self,token_network):
    # Initialize new data flag
    new_data = False

    # Check if the assembly has been changed
    if self.assembly != token_network.assembly:
      self.assembly = self.token_network.assembly
      new_data = True

    # Update the stored token network
    self.token_network = token_network

    # Check if there is new data, or alternatively, if this is just a display update
    # (This prevents infinite callback looping)
    if new_data:
      # Refilter the network
      self.filter_network()

  # Iterate over the network and hilight nodes appropriately
  def filter_network(self):
    # Create entity list from token_network assembly
    entities = self.token_network.token_nodes.keys() + self.token_network.rule_nodes.keys()

    # Initialize matched bool
    matched = False

    # Iterate over all entities
    if len(self.filters) > 0:
      for entity in entities:
	# Iterate over all filters
	for filter in self.filters:
	  # Each filter has to match one property of each entity
	  matched = False

	  matched = matched or -1 != str(entity.name).find(filter)
	  matched = matched or -1 != str(entity.key).find(filter)

	  # Check variables
	  for var in entity.vars:
	    matched = matched or -1 != str(var.name).find(filter)
	    matched = matched or -1 != str(var.key).find(filter)
	    matched = matched or -1 != str(var.domain).find(filter)
	    matched = matched or -1 != str(var.values).find(filter)
	    matched = matched or -1 != str(var.type).find(filter)
	  
	  # Break if any filter does not match
	  if not matched:
	    break
	  
	# Set hilight based on match value
	self.token_network.hilight(entity,matched)

      # Re-post a notification to the network
      self.token_network.notify_listeners()


# Unit tests
class TestTokenNetworkFilter(unittest.TestCase):
  def setUp(self):
    pass
    
  def tearDown(self):
    pass

  def test_populated_network(self):
    from assembly import construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create token network
    network = TokenNetwork()

    # Create token network filter
    network_filter = TokenNetworkFilter()

    # Register network listener
    network.register_listener(network_filter.network_listener)

    # Update assembly
    network.set_assembly(assembly)

    # Add a filter
    network_filter.add_filter("merged")

    network_filter.rem_filter("merged")
    

if __name__ == "__main__":
  unittest.main()
