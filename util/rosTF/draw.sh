#!/usr/bin/env sh

./viewTF && dot -Tps viewTF.dot -o viewTF.dot.ps && evince viewTF.dot.ps