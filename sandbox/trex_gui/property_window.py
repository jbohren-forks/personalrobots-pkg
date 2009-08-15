#!/usr/bin/env python

# System modules
import sys,os
import unittest
import threading, time
import gtk

# Add local python path
sys.path.insert(0, os.path.abspath("./ext"))
# GTKCodeBuffer for doing syntax hilighting for NDDL file display
from gtkcodebuffer import CodeBuffer, SyntaxLoader, add_syntax_path
add_syntax_path(os.path.abspath("./ext/PyGTKCodeBuffer-1.0-RC2/syntax"))

# TREX modules
from assembly import Assembly,Entity,Rule,Token,Slot,Variable

##############################################################################
# PropertyWindow
#   This window provides a user with a list of token or rule variables.
##############################################################################

# Spawn a property window
def PropertyWindowFactory(assembly,entity):
  if entity not in PropertyWindow.window_dict:
    # Create new window
    property_window = PropertyWindow(assembly,entity)
    # Add it to the dict
    PropertyWindow.window_dict[entity] = property_window
  else:
    PropertyWindow.window_dict[entity].present()

  return PropertyWindow.window_dict[entity]

class PropertyWindow(gtk.Window):
  # Declare a list of property windows
  window_dict = {}
    
  def __init__(self,assembly,entity):
    super(PropertyWindow,self).__init__()

    self.set_size_request(500, 400)
    self.set_position(gtk.WIN_POS_CENTER)

    # Remove this property window from the static store
    self.connect("destroy", self.on_destroy)

    # Store data structures
    self.assembly = assembly
    self.entity = entity

    self.set_title(str(self.entity))

    #########################################

    vpaned = gtk.VPaned()

    vbox = gtk.VBox(False, 8)

    sw = gtk.ScrolledWindow()
    sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
    sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)

    vbox.pack_start(sw, True, True, 0)

    store = self.create_model()

    treeView = gtk.TreeView(store)
    treeView.connect("row-activated", self.on_activated)
    treeView.set_rules_hint(True)
    sw.add(treeView)

    self.create_columns(treeView)

    ###################################################

    ###################################################

    if assembly.rule_src.has_key(self.entity.name):
      rule_name = self.entity.name

      # Create the syntax hilighter and code buffer
      lang = SyntaxLoader("cpp")
      buff = CodeBuffer(lang=lang)

      hbox = gtk.HBox(False, 8)
      scr = gtk.ScrolledWindow()
      scr.add(gtk.TextView(buff))
      scr.set_shadow_type(gtk.SHADOW_ETCHED_IN)
      scr.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)	      

      # Get the source file path, and start line
      src_path = assembly.rule_src[rule_name][0]
      src_line_start = int(assembly.rule_src[rule_name][1])-1

      # Open the source file
      src_file = open(src_path,'r')
      src_file_lines = src_file.readlines()
      src_line_end = len(src_file_lines)
      
      # Find the line on which the next rule is defined
      for line in range(src_line_start+1,len(src_file_lines)):
	if src_file_lines[line].find("::") != -1:
	  src_line_end = line;
	  break

      # Display the source code of just this rule
      srccode = "".join(src_file_lines[src_line_start:src_line_end])

      buff.set_text(srccode)
      hbox.pack_start(scr, True, True, 0)
      vbox.pack_start(hbox, True, True, 0)

    ##################################################

    self.statusbar = gtk.Statusbar()
    vbox.pack_start(self.statusbar, False, False, 0)

    self.add(vbox)
    self.show_all()

  def on_destroy(self,widget):
    del PropertyWindow.window_dict[self.entity]

  def create_model(self):
    store = gtk.ListStore(str, str, str, str)

    for var in self.entity.vars:
      store.append([ var.key, var.type, var.name, var.values])
    return store

  def create_columns(self, treeView):
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Key", rendererText, text=0)
    column.set_sort_column_id(0)    
    treeView.append_column(column)
    
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Type", rendererText, text=1)
    column.set_sort_column_id(1)
    treeView.append_column(column)

    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Name", rendererText, text=2)
    column.set_sort_column_id(2)
    treeView.append_column(column)

    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Value", rendererText, text=3)
    column.set_sort_column_id(3)
    treeView.append_column(column)

  def on_activated(self, widget, row, col):
    model = widget.get_model()
    text = model[row][0] + ", " + model[row][1] + ", " + model[row][2]
    self.statusbar.push(0, text)

  def on_srccode_clicked(self, widget):
    if srccode_windows.has_key(self.entity.name):
      srccode_win = srccode_windows[self.entity.name]
      srccode_win.present()	
    else:
      srccode_win = SrccodeWindow(self.entity.name)
      srccode_windows[self.entity.name] = srccode_win
      srccode_win.show()

# Unit tests
class TestPropertyWindow(unittest.TestCase):
  # Create the gtk thread and window structure
  def setUp(self):
    pass
    
  # Destroy window and kill gtk
  def tearDown(self):
    pass

  # Test basic addition and removal of filters
  def test_push_network(self):
    from assembly import construct_test_assembly

    # Create assembly
    assembly = construct_test_assembly()

    # Create a new db reader window
    PropertyWindowFactory(assembly,assembly.tokens.values()[0])
    PropertyWindowFactory(assembly,assembly.tokens.values()[1])
    PropertyWindowFactory(assembly,assembly.tokens.values()[2])
    PropertyWindowFactory(assembly,assembly.tokens.values()[0])

    gtk.main()


if __name__ == '__main__':
  unittest.main()
