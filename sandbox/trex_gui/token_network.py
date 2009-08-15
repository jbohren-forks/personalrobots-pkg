#!/usr/bin/env python

# System modules
import sys,os
import copy
import StringIO
import unittest

# 3rd Party modules
sys.path.insert(0, os.path.abspath("./ext"))
from gvgen import *

# TREX modules
from notifier import Notifier
from assembly import Assembly,Entity,Rule,Token,Slot,Variable

##############################################################################
# TokenNetwork
#   This class provides a mechanism for generating GraphViz DOT code from a
#   EUROPA assembly. This class includes all styling and hilighting methods
#   needed to create a visual graph.
# 
#   During runtime, this class submits a listener callback to a class that can
#   provide an Assembly structure
#
#   Similarly, if an object wants to be notified when a new graph is
#   available, said object can submit a callback method to a TokenNetwork.
#
#   When non-structural changes are made to the graph (hilighting etc) you can
#   signal a new graph by calling token_network.notify_listeners(). This will
#   post an update to all listeners.
##############################################################################

class TokenNetwork(Notifier):
  def __init__(self):
    Notifier.__init__(self)
    # Create dicts for storing graph node structures
    # These dictionaries store key,value = entity instance, node
    # as opposed to the Assembly which stores entity id, entity instance
    self.slot_nodes = {}
    self.token_nodes = {}
    self.rule_nodes = {}

    # Initialize assembly store
    self.assembly = Assembly()

    # Initialize listener list
    self.listeners = []

    # Create a new graph and populate it
    self.create_graph()

  # Create a new graph
  def create_graph(self):
    # Create a new graph with GvGen
    self.graph = GvGen()
    # Create graph styles
    self.define_graph_styles()
    # Populate graph from new data
    self.populate_graph()
    # Notify listeners that new data is available
    self.notify_listeners()

  ############################################################################
  # Node style functions
  ############################################################################

  # Define the styles used by the graph
  def define_graph_styles(self):
    self.graph.styleAppend("Token", "shape", "rectangle")
    self.graph.styleAppend("Token", "style", "filled")
    self.graph.styleAppend("Token", "fillcolor", "#EEEEEE")

    self.graph.styleAppend("TokenHilight", "shape", "rectangle")
    self.graph.styleAppend("TokenHilight", "style", "filled")
    self.graph.styleAppend("TokenHilight", "color", "#8F5E03")
    self.graph.styleAppend("TokenHilight", "fillcolor", "#FFB223")
    self.graph.styleAppend("TokenHilight", "fontcolor", "#000000")

    self.graph.styleAppend("PlannedToken", "shape", "rectangle")
    self.graph.styleAppend("PlannedToken", "style", "filled")
    self.graph.styleAppend("PlannedToken", "fillcolor", "#666666")
    self.graph.styleAppend("PlannedToken", "fontcolor", "#CCCCCC")

    self.graph.styleAppend("MergedToken", "shape", "rectangle")
    self.graph.styleAppend("MergedToken", "style", "filled")
    self.graph.styleAppend("MergedToken", "fillcolor", "#CCCCCC")
    self.graph.styleAppend("MergedToken", "fontcolor", "#444444")

    self.graph.styleAppend("InactiveToken", "shape", "rectangle")
    self.graph.styleAppend("InactiveToken", "style", "filled")
    self.graph.styleAppend("InactiveToken", "fillcolor", "#FFFFFF")
    self.graph.styleAppend("InactiveToken", "fontcolor", "#444444")

    self.graph.styleAppend("Rule", "shape", "diamond")
    self.graph.styleAppend("Rule", "style", "rounded,filled")
    self.graph.styleAppend("Rule", "fillcolor", "#95AEBC")
    self.graph.styleAppend("Rule", "color", "#476170")

    self.graph.styleAppend("RuleHilight", "shape", "diamond")
    self.graph.styleAppend("RuleHilight", "style", "rounded,filled")
    self.graph.styleAppend("RuleHilight", "color", "#8F5E03")
    self.graph.styleAppend("RuleHilight", "fillcolor", "#FFB223")
    self.graph.styleAppend("RuleHilight", "fontcolor", "#000000")

    self.graph.styleAppend("Slot", "shape", "rectangle")
    self.graph.styleAppend("Slot", "style", "rounded,filled")
    self.graph.styleAppend("Slot", "color", "#CCCCCC")
    self.graph.styleAppend("Slot", "fontcolor", "#444444")

  # User data string serializatin 
  def serialize(self,entity):
    if entity.__class__ == Token:
      return "Token:%d" % entity.key
    elif entity.__class__ == Rule:
      return "Rule:%d" % entity.key

  # User data string deserialization
  def deserialize(self,string):
    st = string.split(":")
    if st[0] == "Token":
      return self.assembly.tokens[int(st[1])]
    elif st[0] == "Rule":
      return self.assembly.rules[int(st[1])]

  # Populate the gvgen graph from the stored assembly
  def populate_graph(self):
    # Create subgraphs for each slot
    for sid,slot in self.assembly.slots.iteritems():
      if sid != -1:
	self.slot_nodes[slot] = self.graph.newItem(str(slot));
	self.hilight(slot,False)

    # Create nodes for each token
    for tid,token in self.assembly.tokens.iteritems():
      if token.slot_id != -1:
	# Make this node part of a slot subgraph
	self.token_nodes[token] = self.graph.newItem(str(token), self.slot_nodes[self.assembly.slots[token.slot_id]]);
      else:
	# This node is not in a slot
	self.token_nodes[token] = self.graph.newItem(str(token));

      # Set the user data to link up clickthroughs
      self.graph.propertyAppend(self.token_nodes[token],"URL",self.serialize(token))
      self.hilight(token,False)

    # Create nodes for each rule
    for rid,rule in self.assembly.rules.iteritems():
      self.rule_nodes[rule] = self.graph.newItem(str(rule));

      # Link this rule to it's token
      self.graph.newLink(self.token_nodes[rule.token],self.rule_nodes[rule])
      for token in rule.slaves:
	# Link this rules slaves to itself
	self.graph.newLink(self.rule_nodes[rule],self.token_nodes[token])

      # Set the user data to link up clickthroughs
      self.graph.propertyAppend(self.rule_nodes[rule],"URL",self.serialize(rule))
      self.hilight(rule,False)

  # Toggle hilighting on a node
  def hilight(self, entity, enable):
    style = "None"

    # Switch style type base on node
    if entity.__class__ == Token:
      if enable:
	style = "TokenHilight"
      else:
	if entity.slot_id != -1:
	  if entity.slot_index == 0:
	    style = "Token"
	  else:
	    style = "MergedToken"
	else:
	  style = "InactiveToken"
	if entity.start > self.assembly.tick:
	  style = "PlannedToken"
      # Set style
      self.graph.styleApply(style, self.token_nodes[entity])
    elif entity.__class__ == Rule:
      style = "RuleHilight" if enable else "Rule"
      self.graph.styleApply(style, self.rule_nodes[entity])
    elif entity.__class__ == Slot:
      style = "Slot"
      self.graph.styleApply(style, self.slot_nodes[entity])
  
  # Function to reset the hilighting for the entire graph
  def clear_hilights(self):
    # Create list of all node entities
    entities = self.token_nodes.values() + self.rule_nodes.values()

    for entity in entities:
      self.hilight(entity,False)

  ############################################################################
  # Hooks for setting the assembly used by the token network
  ############################################################################

  # A function to push the assembly
  def set_assembly(self,assembly):
    # replace current assembly with new assembly
    self.assembly = assembly
    # create a new graph
    self.create_graph()

  ############################################################################
  # Dotcode Accessor
  ############################################################################

  # A function to push the assembly
  def get_dotcode(self):
    # Copy the graph for drawing
    # We do this becasue calling dot() on a GvGen graph is destructive
    graph_to_dot = copy.deepcopy(self.graph)

    # Crete a string stream
    dotfile = StringIO.StringIO()
    # Generate dotfile
    graph_to_dot.dot(dotfile)
    # Copy the sstream to a string
    dotstring = dotfile.getvalue()
    dotfile.close()

    return dotstring

# Unit tests
class TestTokenNetwork(unittest.TestCase):
  def setUp(self):
    self.network = TokenNetwork()
    
  def tearDown(self):
    pass

  def test_empty_assembly(self):
    assembly = Assembly()
    self.network.set_assembly(assembly)
    self.assert_(len(self.network.slot_nodes) == 0)
    self.assert_(len(self.network.rule_nodes) == 0)
    self.assert_(len(self.network.token_nodes) == 0)

  def test_populated_assembly(self):
    from assembly import construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create token network
    token_network = TokenNetwork()
    token_network.set_assembly(assembly)

if __name__ == "__main__":
  unittest.main()
