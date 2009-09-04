#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt, re
from pyclearsilver.log import *

import neo_cgi, neo_util, neo_cs

from pyclearsilver import CSPage
from pyclearsilver import odb

import MBPage
import db_webui

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    self.db = db_webui.initSchema()

  def display(self, hdf):
    db_webui.grabTopics(hdf, [])

    hdf.setValue("CGI.now", str(time.time()))

    apps = self.db.apps.fetchAllRows()
    prefix = "CGI.cur.apps"
    i = 0
    for app in apps:
      i = i + 1
      aprefix = prefix + ".%d" % i
      app.hdfExport(aprefix, hdf)
      app.fetchApp(aprefix, hdf)
      
def run(context):
  return MyPage(context, pagename="apps", nologin=1)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
