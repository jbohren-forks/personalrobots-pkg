#!/usr/bin/env python
import rostools.scriptutil as s
s.get_param_server().setParam('/', '/robotdesc/pr2', open('pr2_arm.xml').read())
