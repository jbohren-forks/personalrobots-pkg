#!/usr/bin/env python

# System modules
import sys,os
import unittest
import threading, time
import math

# TREX modules
from notifier import Notifier
from assembly import Assembly,Entity,Rule,Token,Slot,Variable,Object

##############################################################################
# Timeline
#   This class represents a Timeline, a special case of an Object. A timeline
#   holds an ordered sequence of Tokens. This class is defined in the context
#   of a TREX timeline which is either internal or external to a reactor.
##############################################################################

class Timeline():
  # Define internal,external values
  INTERNAL,EXTERNAL = range(2)

  def __init__(self,obj,mode,reactor_name):

    self.mode = mode
    self.reactor_name = reactor_name

    # Ordered list of tokens
    self.tokens = []
    

##############################################################################
# Reactor 
#   This class is used to represent a partial plan as a series
#   of timelines in a more efficient way than an assembly.
##############################################################################

class Reactor():
  def __init__(self,name):
    self.name = name
    # Initialize static assembly for this reactor
    self.assembly = Assembly()
    # Initialize timeline dict timelines[tid] = Timeline()
    self.timelines = {}

##############################################################################
# AgentPlan
#   This class represents the structure of a set of partial plans from all of
#   the reactors in a given agent. It loads in an Assembly structure and
#   maintains a representation of the current and past partial plan.
##############################################################################

class AgentPlan(Notifier):
  def __init__(self):
    # Initialize dict of reactors
    self.reactors = {}

    # Initialize static assembly
    self.assemblies = {}

  # Load new assemblies 
  def set_assemblies(self,assemblies,reactor_name):
    # Set new assemblies
    self.assemblies = assemblies

    # Notify listeners that assemblies have been updated
    self.notify_listeners()

    
# Unit tests
class TestAssemblyStructures(unittest.TestCase):
  def test_construct(self):
    construct_test_assembly()

if __name__ == '__main__':
  unittest.main()
