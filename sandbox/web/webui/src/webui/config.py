
import os

# config data
gRequireUsername = 0
gDefaultModule = "webui"
gDefaultPage = "webui"

gWebUser = "apache"
gWebUserID = None  # apache
gWebGroupID = None # apache

gBaseURL = "/webui/"

gAuthVCode = 1574444059
gAuthSalt = "ir"

gWebUserID = 33  # apache
gWebGroupID = 33 # apache

_path,_fn = os.path.split(__file__)
gDBPath = os.path.join(_path, "..", "..", "db")

gDomain = "willowgarage.com"

LOGIN_TIMEOUT = 60*60*4

# 1 hour
REFRESH_COOKIE_TIMEOUT = 60*60


def getSiteDBPath(module):
  path = os.path.join(gDBPath, module)
  return path
  

def getDBPath(module):
  path = os.path.join(gDBPath, module)
  return path
  
def createDBPath(path):
  if not os.path.isdir(path):
    os.makedirs(path, 0700)
    #webChown(path)
  
def webChown(path):
  if gWebUserID is not None and gWebGroupID is not None:
    os.chown(path, gWebUserID, gWebGroupID)
  
  
