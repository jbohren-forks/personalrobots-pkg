#!/usr/bin/env python

# System modules
import unittest

# TREX modules
from assembly import Assembly,Token

class Timeline():
  # Timeline constants
  INTERNAL, EXTERNAL = range(2)

  def __init__(self,key,name,mode):
    self.key = key
    self.name = name
    self.mode = mode

    self.tokens = []

class DbCore():
  def __init__(self,tick=0,reactor_name=""):
    # Create storage structures
    self.clear()

    # Initialize properties
    self.tick = tick
    self.reactor_name = reactor_name

  def clear(self):
    # Clear timelines
    self.int_timelines = {}
    self.ext_timelines = {}

    # Clear assembly
    self.assembly = Assembly()

