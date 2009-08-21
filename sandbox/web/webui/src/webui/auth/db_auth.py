#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import nstart
import os, sys, string, time, getopt

PKG = 'webui' # this package name
import roslib; roslib.load_manifest(PKG) 

from pyclearsilver.log import *

import config

from pyclearsilver import odb, hdfhelp, odb_sqlite3
from pyclearsilver import CSPage

from pyclearsilver.odb import *

import pwauth

gDBSubPath = "host"
gDBFilename = "auth"
gDBTablePrefix = "auth"

class AuthDB(odb.Database):
  def __init__(self,db,debug=0):
    odb.Database.__init__(self, db, debug=debug)

    self.addTable("users", gDBTablePrefix + "_users", UserTable,
		  rowClass=UserRecord)
    self.addTable("login", gDBTablePrefix + "_login", UserLoginTable)
    self.addTable("vcode", gDBTablePrefix + "_vcode", VCodeTable)
    self.addTable("browserid", gDBTablePrefix + "_browserid", BrowserTable)

  def defaultRowClass(self):
    return hdfhelp.HdfRow
  def defaultRowListClass(self):
    return hdfhelp.HdfItemList

  def getAllUsers(self):
    users = []
    rows = self.users.fetchAllRows()
    for row in rows:
      users.append(row.username)

    return users
    



class UserTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("uid",kInteger,None,primarykey = 1,
                     autoincrement = 1)

    self.d_addColumn("username",kVarString, indexed=1, unique=1)
    self.d_addColumn("role", kVarString, default="")
    self.d_addColumn("pw_hash",kVarString)
    self.d_addColumn("status",kInteger, default=0)
    self.d_addColumn("creationDate", kInteger, default=0)

  def lookup(self, username):
    try:
      row = self.fetchRow(('username', username))
    except odb.eNoMatchingRows, reason:
      row = None
    return row

  def new(self, username, password):
    row = self.lookup(username)
    if row is not None: return row

    row = self.newRow()
    row.username = username
    row.creationDate = int(time.time())
    row.setPassword(password)
    row.save()

    return row
    
class UserRecord(hdfhelp.HdfRow):
  def checkPasswordHash(self, passwordHash):
    if len(self.pw_hash) < 2: return 0
    if passwordHash == self.pw_hash: return 1
    return 0


  def checkPassword(self, password):
    if len(self.pw_hash) < 2: return 0

    return pwauth.checkPassword(password, self.pw_hash)

  def setPassword(self, new_password):
    self.pw_hash = pwauth.cryptPassword(new_password)
    self.save()

class UserLoginTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("uid",kInteger, primarykey=1)
    self.d_addColumn("username",kVarString, indexed=1, primarykey=1)
    self.d_addColumn("time", kCreatedStampMS, primarykey=1)

    self.d_addColumn("loginType", kInteger)   
    # 0 - incorrect password
    # 1 - correct password

    self.d_addColumn("browserid",kVarString)
    self.d_addColumn("ipaddr",kVarString)


class VCodeTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("username",kVarString, primarykey=1)
    self.d_addColumn("vcode",kInteger, default=0)
    self.d_addColumn("browserid",kInteger, default=0)
    self.d_addColumn("creationDate", kInteger, default=0)


class BrowserTable(odb.Table):
  def _defineRows(self):
    self.d_addColumn("browserid",kInteger, primarykey=1, autoincrement=1)
    self.d_addColumn("ipaddr", kVarString)
    self.d_addColumn("creationDate", kInteger, default=0)

    

def fullDBPath(path_to_store):
  return os.path.join(path_to_store, gDBFilename + ".db3")

def initSchema(create=0, timeout=None):
  if timeout is None: timeout = 600

  path = config.getSiteDBPath(gDBSubPath)

  if create == 1:
    config.createDBPath(path)

  conn = odb_sqlite3.Connection(fullDBPath(path),
                               timeout=timeout)

  db = AuthDB(conn,debug=debug)

  if create:
    db.createTables()
    db.synchronizeSchema()
    db.createIndices()

    if 0:
      if config.gWebUserID is not None and config.gWebGroupID is not None:
        config.webChown(fullDBPath(path))

  return db

def exists(username):
  path = config.getSiteDBPath(gDBSubPath)
  fn = fullDBPath(path)
  if os.path.exists(fn): 
    return 1
  return 0
  

def createDB():
  db = initSchema(create=1)
  return db
  
  
def test():
  db = initSchema()

  rows = db.users.fetchAllRows()
  for row in rows:
    print row.username, row.pw_hash



def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return

  db = initSchema(create=1)



if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)

  
  
