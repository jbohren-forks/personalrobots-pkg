#!/usr/bin/env sh

cd `rospack find rosTF` && ./viewTF --dump viewTF.dot && dot -Tps viewTF.dot -o viewTF.dot.ps && evince viewTF.dot.ps
