#!/usr/bin/python
import gtk
import sys
import os.path
import pango

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


# additional style definitions:
#   the update_syntax() method of CodeBuffer allows you to define new and modify
#   already defined styles. Think of it like CSS.
styles = { 'DEFAULT':   {'font': 'serif'},
           'bold':      {'weight': 700},
           'comment':   {'foreground': 'gray',
                         'weight': 700},
           'heading':   {'variant': pango.VARIANT_SMALL_CAPS,
                         'underline': pango.UNDERLINE_DOUBLE} }

# Syntax definition
emph  = String(r"\*", r"\*", style="comment")
emph2 = String(r"\*\*", r"\*\*", style="bold")
code  = String(r'`', r'`', style="special")
head  = Pattern(r"^#+.+$", style="heading")
list1 = Pattern(r"^(- ).+$", style="comment", group=1)
list2 = Pattern(r"^(\d+\. ).+$", style="comment", group=1) 

# create lexer: 
lang = LanguageDefinition([emph, emph2, code, head, list1, list2])

# create buffer and update style-definition 
buff = CodeBuffer(lang=lang, styles=styles)

win = gtk.Window(gtk.WINDOW_TOPLEVEL)
scr = gtk.ScrolledWindow()
win.add(scr)
scr.add(gtk.TextView(buff))
        
win.set_default_size(300,200)
win.show_all()
win.connect("destroy", lambda w: gtk.main_quit())

buff.set_text(txt)
        
gtk.main()        
