#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart

import os, sys, string, time, getopt, re
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb

import MBPage
import db_webui

import roslib
import roslib.scriptutil

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    db_webui.grabTopics(hdf, ["/topics"])

def run(context):
  return MyPage(context, pagename="nodes", nologin=1)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
