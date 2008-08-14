#!/usr/bin/env sh

./viewTF --dump viewTF.dot && dot -Tps viewTF.dot -o viewTF.dot.ps && evince viewTF.dot.ps
