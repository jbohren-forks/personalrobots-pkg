#! /usr/bin/env python

import os, sys
import nstart

from pyclearsilver import cgistarter

import config

path,f = os.path.split(__file__)
#sys.stderr.write("path: %s\n"  % path)
os.chdir(path)

cgistarter.setConfig(config)
handler = cgistarter.handler

