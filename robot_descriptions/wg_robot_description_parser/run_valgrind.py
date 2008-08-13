#!/usr/bin/env python
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
