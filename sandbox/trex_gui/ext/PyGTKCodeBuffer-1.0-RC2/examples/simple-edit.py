#!/usr/bin/python

import gtk
import gtk.glade
import os
import glob
import os.path
import re

# not needed if you have codebuffer installed:
import sys
sys.path.insert(0, "..")

from gtkcodebuffer import CodeBuffer, SyntaxLoader, add_syntax_path

# also not needed if installed:
add_syntax_path("../syntax/")




class Editor:
    def __init__(self):
        self.__gladefile = os.path.join(os.path.dirname(__file__), "simpleedit.glade")
        self.__xml = gtk.glade.XML(self.__gladefile, "mainwindow")
        
        synmenu = self.__xml.get_widget("syntaxmenu")
        lst = glob.glob("../syntax/*.xml")
        lst.sort()
        lst = map(lambda x: re.match("^(.+)\.xml$",os.path.basename(x)).group(1), lst)
        
        for lang in lst:
            item = gtk.MenuItem(lang, False)
            synmenu.append(item)
            item.connect("activate", self.on_lang_changed, lang)
        
        self.__xml.get_widget("fileopen").connect("activate", self.on_open)
        self.__xml.get_widget("helpinfo").connect("activate", self.on_show_info)
        self.__xml.get_widget("mainwindow").show_all()
        self.__xml.get_widget("mainwindow").connect("destroy",self.on_destroy)

        self.__buffer = CodeBuffer(None)
        self.__xml.get_widget("textview").set_buffer(self.__buffer)
        
            
    def on_lang_changed(self, widget, lang):
        lspec = SyntaxLoader(lang)
        self.__buffer.reset_language(lspec)
    
    
    def on_open(self, widget):
        dlg = gtk.FileSelection("Open...")
        
        if not dlg.run() == gtk.RESPONSE_OK:
            dlg.destroy()
            return
        
        fname = dlg.get_filename()
        dlg.destroy()
        
        self.__buffer.set_text(open(fname, "r").read())
        
        
    def on_show_info(self, widget):
        dlg = gtk.glade.XML(self.__gladefile, "aboutdialog").get_widget("aboutdialog")
        dlg.run()
        dlg.destroy()
        
            
    def on_destroy(self, widget):
        gtk.main_quit()
        
        
        
            
if __name__ == '__main__':
    edit = Editor()
    gtk.main()        
