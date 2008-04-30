#!/usr/bin/env ruby
require "#{`#{ENV['ROS_ROOT']}/rospack latest yamlgraph`}/lib/yamlgraph/ygl.rb"
g = YAMLGraph.new
#g.param 'pinger.msg', ' -'
#g.param 'pinger.freq', 10.0
#g.param 'ponger.msg', ' *'
g.node 'erratic_player/erratic_player'
g.node 'linuxjoystick_player/linuxjoystick_player'
g.flow 'linuxjoystick_player:cmdvel', 'erratic_player:cmdvel'
YAMLGraphLauncher.new.launch g
