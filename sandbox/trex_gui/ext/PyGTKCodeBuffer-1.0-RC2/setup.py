#!/usr/bin/python


from distutils.core import setup
import os.path
import glob 

loc_dir      = os.path.dirname(__file__)
syntax_files = glob.glob(os.path.join(loc_dir, "syntax","*.xml"))

desc = """PyGTKCodeBuffer - Lightweight syntax-highlighting for PyGTK's TextView-widget."""

long_desc = """PyGTKCodeBuffer is a syntax-highlighting engine written in pure 
Python to provide maximum portability. It depends only on PyGTK and the Python 
standard library. No Gnome nor Scintilla libraries are needed so it should run 
perfectly under all platforms supported by PyGTK!"""

setup ( name = 'PyGTKCodeBuffer',
        version = '1.0-RC2',
        description = desc,
        long_description = long_desc,
        author = 'Hannes Matuschek',
        author_email = 'hmatuschek@gmail.com',
        url = 'http://pygtkcodebuffer.googlecode.com',
        download_url = 'http://pygtkcodebuffer.googlecode.com/files/PyGTKCodeBuffer-1.0-RC1.tar.bz2',
        
        classifiers = ['Development Status :: 5 - Production/Stable',
                       'Environment :: X11 Applications :: GTK',
                       'Intended Audience :: Developers',
                       'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
                       'Operating System :: OS Independent',
                       'Programming Language :: Python',
                       'Topic :: Software Development :: Libraries :: Python Modules',
                       'Topic :: Software Development :: Widget Sets',
                       'Topic :: Text Editors'], 

        py_modules = ['gtkcodebuffer'],
        
        data_files = [('share/pygtkcodebuffer/syntax', syntax_files)]
      )                
