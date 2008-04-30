all: gen_msgs
clean: clean_msgs
include $(shell rospack find mk)/msg.mk

SUBDIRS = src
include $(shell rospack find mk)/recurse_subdirs.mk
