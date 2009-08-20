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
    hdf.setValue("CGI.now", str(time.time()))

    rows = self.db.apps.fetchAllRows()
    rows.hdfExport("CGI.cur.apps", hdf)

def run(context):
  return MyPage(context, pagename="apps", nologin=1)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()
