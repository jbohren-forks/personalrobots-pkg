#!/usr/bin/env python

import nstart
import config
import sys
import os
import re

import MinibooMailbox

from pyclearsilver.CSPage import Context
from MBPage import MBPage

from pyclearsilver.log import *

gInvalidLogin = {}
gInvalidLogin['support'] = 1
gInvalidLogin["mailer-daemon"] = 1
gInvalidLogin["news"] = 1
gInvalidLogin["root"] = 1
gInvalidLogin["mail"] = 1
gInvalidLogin["sync"] = 1
gInvalidLogin["shutdown"] = 1
gInvalidLogin["games"] = 1
gInvalidLogin["operator"] = 1
gInvalidLogin["nobody"] = 1

class RegisterPage(MBPage):
    def display(self):
        hdf = self.ncgi.hdf

        q_login = hdf.getValue("Query.login","")
        q_login = q_login.lower()
        hdf.setValue("CGI.Login",q_login)

        if hdf.getValue("Query.st","") == "done":
          hdf.setValue("CGI.mailAddrPattern", config.gMailAddrPattern % q_login)
          
          self.pagename = "register_confirm"

    def Action_Create(self):
        hdf = self.ncgi.hdf
        q_login = hdf.getValue("Query.login","")
        q_pw1   = hdf.getValue("Query.pw1","")
        q_pw2   = hdf.getValue("Query.pw2","")
        q_outaddr = hdf.getValue("Query.out_address","")


        if q_login and len(q_login) >= 4:
          if not re.match("[a-zA-Z_][a-zA-Z_0-9]+$",q_login):
            self.redirectUri("register.py?err=Invalid+Login+Format")

        if gInvalidLogin.has_key(q_login):
            self.redirectUri("register.py?err=Login+Taken")

        hdf.setValue("CGI.Login",q_login)

        if not q_pw1 or (q_pw1 != q_pw2):
            self.redirectUri("register.py?err=Password+Mismatch")

        if len(q_pw1) < 4:
            self.redirectUri("register.py?err=Password+Too+Short")

	import userCtl
	userCtl.newUser(q_login, q_pw1)

        self.redirectUri("register.py?st=done&login=%s" % q_login)
        
        

def run(context):
    return RegisterPage(context, pagename="register_1",nologin=1)

def main(context):
  run(context).start()

if __name__ == "__main__":
    main(Context())
