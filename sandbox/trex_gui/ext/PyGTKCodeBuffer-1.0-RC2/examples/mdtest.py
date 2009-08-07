#!/usr/bin/python
import gtk

# not needed if you have codebuffer installed:
import sys
sys.path.insert(0, "..")

from gtkcodebuffer import CodeBuffer, SyntaxLoader, add_syntax_path

# also not needed if installed:
add_syntax_path("../syntax/")


txt = """
# About 
This example shows you a markdown with XML file
syntax-definition. Supporting `code-segments`, 
**emphasized text** or *emphasized text*. 

Alterative emphasis style:
_single underscores_ and __double underscores__.
Note that a variable like some_stuff_list is not _emphasized_. 
See "Middle-World Emphasis" at 
<http://www.freewisdom.org/projects/python-markdown/Features> 
along with <http://six.pairlist.net/pipermail/markdown-discuss/2005-October/001610.html>


This is simply multiplication, 3 * 23 + constant * 11 = x.
I.e. it is not emphasized.


For more Markdown information see URLs <http://en.wikipedia.org/wiki/Markdown>
and <http://daringfireball.net/projects/markdown/>


## list-support
- a simple list item
+ an other style
* yet other list/bullet style

  1. A ordered list
  2. other item

#### n-th order heading

## Indents/code
This demos the other code markup style, space indentation:

    this is pre-formated text
    as is this.

Indentation with tabs:

\tthis is pre-formated text
\tas is this. Prefixed with tab

Next is quoting (indentation):

> This is quoted text,
> as is this.


Underline headings
==================
And Underline headings
----------------------
are supported since version 0.3.5
also this [link] type


## TODO and Unsupported Markup

TODO! I've overloaded some of the styles/colours in the XML file,
e.g. URLs are datatypes (like emphasis).

Currently does not handle:

*   Horizontal Rules/Lines

"""


lang = SyntaxLoader("markdown")
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
