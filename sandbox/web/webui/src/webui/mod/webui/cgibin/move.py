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
    self.db = db_webui.initSchema()
    
  def display(self, hdf):
    rez = hdf.getValue("Query.rez", "0.075")
    hdf.setValue("CGI.cur.rez", rez)
    return


def run(context):
  return MyPage(context, pagename="move", nologin=1)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
