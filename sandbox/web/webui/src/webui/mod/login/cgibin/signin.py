#!/usr/bin/env python

import nstart
import config
import os, sys, string, time

from pyclearsilver.log import *

from pyclearsilver.CSPage import Context
import neo_cgi, neo_cs, neo_util
from MBPage import MBPage

from auth import browserauth
from auth import cookieauth
from auth import db_auth
from auth import pwauth

from pyclearsilver import wordwrap
from email import MIMEText, Generator, Parser
from cStringIO import StringIO

class SignInPage(MBPage):
    def setup(self, hdf):
      self.requestURI = hdf.getValue("Query.request", "")
      self.authdb = db_auth.initSchema()

    def display0(self, hdf):
        q_signout = hdf.getIntValue("Query.signout",0)
        self.requestURI = hdf.getValue("Query.request", "")
        if self.requestURI:
          hdf.setValue("CGI.cur.request", self.requestURI)

        if q_signout:
          cookieauth.clearLoginCookie(self.ncgi, self.username)

    def display(self, hdf):
      wwwhostname = hdf.getValue("HTTP.Host", "")
      domain = wwwhostname

      q_signout = hdf.getIntValue("Query.signout",0)
      if q_signout:
        cookieauth.clearLoginCookie(self.ncgi, self.username, domain)

      rurl = self.http + wwwhostname + config.gBaseURL + "%s/" % config.gDefaultModule

      self.redirectUri(rurl)
      

    def Action_Login(self):
        hdf = self.ncgi.hdf

        q_username =  hdf.getValue("Query.username","")
        q_passwordHash = hdf.getValue("Query.password","")
        q_persist = hdf.getValue("Query.persist","0")

        try: q_persist = int(q_persist)
        except ValueError: q_persist = 0

##        if not self.requestURI:
##          self.requestURI = config.gBaseURL + q_username + "/mail/topfrm.py?q=1"

        default_requestURI = config.gBaseURL + "%s/" % config.gDefaultModule

        warn("requestURI", self.requestURI)

        if not self.requestURI:
          self.requestURI = default_requestURI


        wwwhostname = hdf.getValue("HTTP.Host", "")

        rurl = self.http + wwwhostname + config.gBaseURL + "login/signin0.py"

        warn("signin.py", rurl)

        # open login db to get pw
        userRec = self.authdb.users.lookup(q_username)
        
        if not userRec:
          warn("signin.py", "login failure (%s) unknown user" % q_username)
          self.redirectUri(rurl + "?err=Invalid+Login&request=%s" % neo_cgi.urlEscape(self.requestURI))

        q_password = pwauth.unmungePassword(q_passwordHash)

        ipaddr = hdf.getValue("CGI.RemoteAddress", "Unknown")
        browserid = browserauth.getBrowserCookie(self.ncgi)

        now = time.time()

        

        loginRow = self.authdb.login.newRow()
        loginRow.uid = userRec.uid
        loginRow.username = userRec.username
        loginRow.ipaddr = ipaddr
        loginRow.browserid = browserid

        if userRec.checkPassword(q_password) == 0:
          warn("signin.py", "login failure (%s) password mismatch" % q_username, q_password)
          loginRow.loginType = 0
          loginRow.save()


          url = rurl + "?err=Invalid+Login&request=%s" % neo_cgi.urlEscape(self.requestURI)
          warn("redirecting to", url)
          self.redirectUri(url)
          return

        # ----------- success!!! ------------------
        # generate cookie

        loginRow.loginType = 1
        loginRow.save()

        cookieauth.issueLoginCookie(self.ncgi, self.authdb, q_username, userRec.pw_hash, q_persist)
        

        # redirect to the main page
        self.redirectUri(self.requestURI)



    def __del__(self):
        if self.authdb:
            self.authdb.close()
            self.authdb = None

def run(context):
    page = SignInPage(context, pagename="signin",nologin=1)
    return page

def main(context):
  page = run(context)
  page.start()
  

if __name__ == "__main__":
    main(Context())
