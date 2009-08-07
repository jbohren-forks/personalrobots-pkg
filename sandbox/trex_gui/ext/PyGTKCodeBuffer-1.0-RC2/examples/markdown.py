#!/usr/bin/python
import gtk
import sys
import os.path

# if you don't have pygtkcodebuffer installed...
sys.path.insert(0, os.path.abspath("./../"))

from gtkcodebuffer import CodeBuffer, Pattern, String, LanguageDefinition 
from gtkcodebuffer import SyntaxLoader, add_syntax_path


txt = """
# About 
This example shows you a hard-coded markdown 
syntax-definition. Supporting `code-segments`, 
**emphasized text** or *emphasized text*.

## list-support
- a simple list item
- an other

1. A ordered list
2. other item

#### n-th order heading
"""


# Syntax definition
emph  = String(r"\*", r"\*", style="datatype")
emph2 = String(r"\*\*", r"\*\*", style="datatype")
code  = String(r'`', r'`', style="special")
head  = Pattern(r"^#+.+$", style="keyword")
list1 = Pattern(r"^(- ).+$", style="comment", group=1)
list2 = Pattern(r"^(\d+\. ).+$", style="comment", group=1)
 
lang = LanguageDefinition([emph, emph2, code, head, list1, list2])

# or load from file... 
#add_syntax_path("./../syntax")
#lang = SyntaxLoader("markdown")

buff = CodeBuffer(lang=lang)

win = gtk.Window(gtk.WINDOW_TOPLEVEL)
scr = gtk.ScrolledWindow()
win.add(scr)
scr.add(gtk.TextView(buff))
        
win.set_default_size(300,200)
win.show_all()
win.connect("destroy", lambda w: gtk.main_quit())

buff.set_text(txt)
        
gtk.main()        
