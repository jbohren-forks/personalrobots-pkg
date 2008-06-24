# This make file just uses the underlying jam system to build everything. It is provided for
# uniformity. Eventually, we may swap out the use of jam for the node.
# See the README.txt file before using.

all: exec_trex_opt exec_trex_debug

exec_trex_debug:
	jam exec_trex

exec_trex_opt:
	jam -sVARIANTS=OPTIMIZED exec_trex
