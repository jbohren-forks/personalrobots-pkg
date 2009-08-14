#!/usr/bin/env python

import unittest

##############################################################################
# Model Classes
#   These are used to represent the different entities inside of EUROPA
##############################################################################

class Entity():
  def __init__(self,key):
    self.key = key
    self.vars = []

class Rule(Entity):
  def __init__(self,key,name,token,filename,line,slaves):
    Entity.__init__(self,key)
    self.name = name
    self.token = token
    self.slaves = slaves
    self.filename = filename
    self.line = line

  def __str__(self):
    return "%s (%d)" % (self.name, self.key)

class Token(Entity):
  def __init__(self,key,name,slot_id,slot_index):
    Entity.__init__(self,key)
    self.name = name
    self.slot_id = slot_id
    self.slot_index = slot_index

  def __str__(self):
    return "%s (%d)" % (self.name, self.key)

class Slot():
  def __init__(self,id,tokens = []):
    self.id = id
    self.tokens = tokens

  def __str__(self):
    return "Slot (%d)" % (self.id)

class Variable():
  def __init__(self, key, name, entity,  domain, values, type):
    self.key = key
    self.entity = entity
    self.name = name 
    self.domain = domain
    self.values = values 
    self.type = type

  def __str__(self):
    return "%s = %s (%s)" % (self.name, self.values, self.key)

##############################################################################
# Assembly
#   This class represents the state of a EUROPA database. 
##############################################################################

class Assembly():
  def __init__(self):
    # Create storage structures
    # Rules are predicates defined in NDDL
    self.rules = {}
    # Tokens are instantiations of rules
    self.tokens = {}
    # Slots are merged sets of tokens
    self.slots = {}
    # Variables are properties of both rules, tokens, objects, and other structures
    self.vars = {}

  def clear(self):
    self.rules = {}
    self.tokens = {}
    self.slots = {}
    self.vars = {}

# Testing code
class TestAssemblyStructures(unittest.TestCase):
  def test_construct(self):
    # Create assembly
    assembly = Assembly()

    # Instantiate some things
    assembly.tokens[0] = Token(
	0,
	"test_token",
	0,
	0)
    assembly.tokens[1] = Token(
	1,
	"test_slave_token",
	1,
	0)
    assembly.slots[0] = Slot(
	0,
	[assembly.tokens[0]])
    assembly.slots[1] = Slot(
	1,
	[assembly.tokens[1]])
    assembly.rules[1] = Rule(
	2,
	"test_rule",
	assembly.tokens[0],
	"/dev/null",
	0,
	[assembly.tokens[1]])
    assembly.vars[0] = Variable(
	3,
	"test_var",
	assembly.tokens[0],
	"BOOL",
	"[TRUE FALSE MAYBE]",
	"ENUMERATED_DOMAIN")

if __name__ == '__main__':
  unittest.main()