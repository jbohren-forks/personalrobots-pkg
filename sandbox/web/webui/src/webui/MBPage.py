
from pyclearsilver.CSPage import CSPage

import time
import string
import crypt
import gc

import nstart
import config

from pyclearsilver.log import *

import neo_cgi
from pyclearsilver import handle_error
from auth import db_auth, cookieauth

# 4 hours
LOGIN_TIMEOUT = 60*60*4

REFRESH_COOKIE_TIMEOUT = 0

class MBPage(CSPage):
    def subclassinit(self):
#        self._pageparms["nologin"] = 1
        hdf = self.ncgi.hdf
#        self.setPaths([config.gTemplatePath])

#        hdf.setValue("Query.debug", "1")
#        hdf.setValue("Config.DebugPassword","1")
        hdf.setValue("Config.CompressionEnabled","0")
        hdf.setValue("Config.WhiteSpaceStrip","0")

        self.login = None
        self.username = None
        self.db = None
        self.userRec = None

        now = int(time.time())
        today = time.localtime(now)
        neo_cgi.exportDate(hdf, "CGI.Today", "US/Pacific", now)

        self.authdb = db_auth.initSchema()
        
        self.getUsername()

        self.setStyleSheet(hdf)

    def setStyleSheet(self, hdf):
      useragent = hdf.getValue("HTTP.UserAgent", "").lower()
      if useragent.find("android") != -1 or useragent.find("iphone") != -1:
        hdf.setValue("CGI.cur.device_style", "style_phone.css")
      else:
        hdf.setValue("CGI.cur.device_style", "style_desktop.css")

    def handle_actions2(self):
      hdf = self.ncgi.hdf
      hdfobj = hdf.getObj("Query.Action")
      if hdfobj:
        self.checkLoginCookie()
      CSPage.handle_actions(self)

    def getUsername(self):
      hdf = self.ncgi.hdf

      logincookie = cookieauth.parseLoginCookie(self.ncgi)
      if logincookie:
        self.username = logincookie.username

        self.userRec = self.authdb.users.lookup(self.username)
        hdf.setValue("CGI.Role", self.userRec.role)

        hdf.setValue("CGI.Login", self.username)
        hdf.setValue("CGI.Login.issued_at", str(logincookie.issued_at))
        ## set the role for administrators
#        if self.username in ("hassan", "steffi", "keenan", "tashana"):
#          hdf.setValue("CGI.Role", "admin")

    def checkLoginCookie(self):
        hdf = self.ncgi.hdf

        requestURI = hdf.getValue("CGI.RequestURI", "")

        rurl = config.gBaseURL + "login/signin0.py"

        self.authdb = db_auth.initSchema()

        logincookie = cookieauth.parseLoginCookie(self.ncgi)
        if not logincookie:
          self.redirectUri(rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))

        self.username = logincookie.username
        userRec = self.authdb.users.lookup(self.username)

        if userRec is None or cookieauth.checkLoginCookie(self.ncgi, logincookie, self.authdb, self.username, userRec) == 0:
          warn("invalid cookie", rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))
          self.redirectUri(rurl + "?q=1&request=%s" % neo_cgi.urlEscape(requestURI))
        # -----  the cookie is valid!!!! -------

        persist = cookieauth.getPersistCookie(hdf)
        if persist == 0:
          # reissue a new cookie with an updated timeout
          if (time.time() - logincookie.issued_at) > config.REFRESH_COOKIE_TIMEOUT:
            cookieauth.issueLoginCookie(self.ncgi, self.authdb, self.username, userRec.pw_hash)

        self.login = self.username

        hdf.setValue("CGI.Login", self.username)
        hdf.setValue("CGI.Login.issued_at", str(logincookie.issued_at))

    def close(self):
      if hasattr(self, "db") and self.db:
        self.db.close()
        self.db = None
      if hasattr(self, "authdb") and self.authdb:
        self.authdb.close()
        self.authdb = None

    def __del__(self):
      self.close()
      gc.collect()

