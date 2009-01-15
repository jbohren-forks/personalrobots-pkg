#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## \Author Ioan Sucan 

from subprocess import Popen, PIPE
import string
import re
import sys

def main(): 
    errors = 0
    leaks = 0
    command = ["valgrind", "--leak-check=full"]
    for arg in sys.argv[1:len(sys.argv)]:
        command.append(arg)

    find_errors = re.compile(".*ERROR SUMMARY: (\d+) errors.*")
    find_leaks = re.compile(".*in use at exit: (\d+) bytes.*")
    P = Popen(command, stdout=PIPE, stderr=PIPE, close_fds=True)
    try:
        for line in P.stderr:
            m = find_errors.match(line)
            if m:
                errors = string.atoi(m.group(1))
            m = find_leaks.match(line)
            if m:
                leaks = string.atoi(m.group(1))
    finally:
        P.stderr.close()

    sys.exit(errors + leaks)

if __name__ == "__main__":
    main()
