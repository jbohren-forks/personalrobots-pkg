#!/usr/bin/env python

# System modules
import sys,os
import re
import csv
import unittest

# TREX modules
from assembly import Assembly,Entity,Rule,Token,Slot,Variable,Object

##############################################################################
# DbReader
#   This class complements the TREX DbWrtier. It loads files written out with
#   the format that the DbWriter uses to export a database state.
##############################################################################

# DbReader
class DbReader():
  ASSEMBLY_PATH = "assembly_dumps"

  def __init__(self):
    # Compile expressions
    self.line_regex = re.compile("^(.+)\,([0-9]+)$")
    self.number_range_regex = re.compile("\[*([^\]]+)\]*")

  def get_reactor_path(self, log_path, reactor_name):
    return os.path.join(log_path,DbReader.ASSEMBLY_PATH,reactor_name)

  def get_available_reactors(self, log_path):
    return os.listdir(os.path.join(log_path,DbReader.ASSEMBLY_PATH))

  def get_available_ticks(self,log_path,reactor_name):
    tick_paths = os.listdir(self.get_reactor_path(log_path,reactor_name))
    ticks = [int(os.path.basename(s)[4:]) for s in tick_paths if s[0:4] == "tick"]
    ticks.sort()
    return ticks

  ############################################################################
  # load_assembly(log_path,reactor_name,tick)
  #   This function takes a file path to load the files relevant to a specific
  #   reactor and tick. The file path is the full path through the reactor.
  #   Note that if the parameters are incorrect, the get_assembly call will
  #   throw an exception.
  # Input:
  #   reactor_path: "../path/to/log", not "../path/to/log/assembly_dumps"
  # Output:
  #   It returns a populated Assembly
  ############################################################################

  def load_assembly(self,log_path,reactor_name,tick):
    # Create a new Assembly for storing data
    assembly = Assembly()

    # Set assembly properties
    assembly.reactor_name = reactor_name
    assembly.tick = tick

    # Generate the reactor_path
    reactor_path = os.path.join(log_path,DbReader.ASSEMBLY_PATH,reactor_name)

    # Generate the step path
    step_path = os.path.join(reactor_path, "tick%d" % tick, "tick%d" % tick)

    # Read in rule source code paths
    rule_src_reader = csv.reader(open(os.path.join(reactor_path,"rules")),delimiter='\t')

    # Put the source code paths and line numbers into a temporary dictionary
    assembly.rule_src = {}
    for row in rule_src_reader:
      rule_name = row[1]
      rule_path_line_str = row[2]

      # Get the rule path and line from the "rules" file
      rule_path_line = self.line_regex.findall(rule_path_line_str)
      assembly.rule_src[rule_name] = rule_path_line[0]

    # Read in tokens
    tokens_reader = csv.reader(open("%s.tokens" % step_path),delimiter='\t')
    for row in tokens_reader:
      token_key = int(row[0])
      predicate_name = row[11]
      slot_id = int(["-1",row[2]][row[2]!='\N'])
      slot_index = int(row[16])

      # Add this token to the dict
      assembly.tokens[token_key] = Token(
	  token_key,
	  predicate_name,
	  slot_id,
	  slot_index)

      # Append token key to slot to which it belongs
      if assembly.slots.has_key(slot_id):
	assembly.slots[slot_id].tokens.append(assembly.tokens[token_key])
      else:
	assembly.slots[slot_id] = Slot(slot_id,[assembly.tokens[token_key]])

    # Read in rule instances
    rule_instances_reader = csv.reader(open("%s.ruleInstances" % step_path),delimiter='\t')
    
    for row in rule_instances_reader:
      rule_key = int(row[0])
      rule_name = row[3]
      rule_token_key = int(row[4])
      slave_token_keys = row[5]
      variable_keys = row[6]

      rule_filename = assembly.rule_src[rule_name][0]
      rule_line = assembly.rule_src[rule_name][1]
      
      # Get rule slaves
      rule_slaves = [assembly.tokens[int(slave)] for slave in slave_token_keys.split(',') if slave != '' and slave != '\N'];

      # Append rule to rule dict
      assembly.rules[rule_key] = Rule(
	  rule_key,
	  rule_name,
	  assembly.tokens[rule_token_key],
	  rule_filename,
	  rule_line,
	  rule_slaves)

    # Read in variables
    vars_reader = csv.reader(open("%s.variables" % step_path),delimiter='\t')
    for row in vars_reader:
      var_key = int(row[0])
      var_token_key = int(row[2])
      var_name = row[3]
      var_domain = row[4]
      var_values = row[5]
      if var_values == '\N':
	var_values = "[%s %s]" % (row[7], row[8])
      var_type = row[9]

      # Add variable reference to token
      entity = None
      if var_type != "MEMBER_VAR":
	if var_type == "RULE_VAR":
	  entity = assembly.rules[var_token_key]
	else:
	  entity = assembly.tokens[var_token_key]

      # Append variable to vars dict
      assembly.vars[var_key] = Variable(
	  var_key,
	  var_name,
	  entity,
	  var_domain,
	  var_values,
	  var_type)

      # Add variables to entity
      if entity:
	entity.vars.append(assembly.vars[var_key])
	# Add the start value if available:
	if var_name == "start":
	  start_bounds = self.number_range_regex.findall(var_values)
	  entity.start = [float(s) for s in start_bounds[0].split(" ")]
	elif var_name == "end":
	  end_bounds = self.number_range_regex.findall(var_values)
	  entity.end = [float(s) for s in end_bounds[0].split(" ")]

    # Read in objects
    obj_reader = csv.reader(open("%s.objects" % step_path),delimiter='\t')
    for row in obj_reader:
      obj_key = int(row[0])
      obj_name = row[4]

      # Parse object variables
      if row[6] == '\N':
	obj_vars = []
      else:
	obj_vars = [assembly.vars[int(key_str)] for key_str in row[6].split(",") if key_str]

      # Parse object tokens 
      if row[7] == '\N':
	obj_tokens = []
      else:
	obj_tokens = [assembly.tokens[int(key_str)] for key_str in row[7].split(",") if key_str]

      # Append variable to vars dict
      assembly.objects[obj_key] = Object(
	  obj_key,
	  obj_name,
	  obj_vars,
	  obj_tokens)

    # Return constructed assembly database
    return assembly

# Unit tests
class TestDbReader(unittest.TestCase):
  def test_read(self):
    # Create a db reader
    db_reader = DbReader()
    # Define the log path
    log_path = "./test/db_reader"
    # Get the available reactor names
    reactor_names = db_reader.get_available_reactors(log_path)
    # Get the available ticks
    ticks = db_reader.get_available_ticks(log_path,reactor_names[0])
    # Load in the assembly from that tick
    assembly = db_reader.load_assembly(log_path,reactor_names[0],ticks[0])

if __name__ == '__main__':
  unittest.main()

