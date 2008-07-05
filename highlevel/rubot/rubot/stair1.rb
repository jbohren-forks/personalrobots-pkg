require(`rospack find rubot`.strip + "/rubot/rubot.rb")

class Stair1 < Rubot
  def initialize
    super
    register_action('deadreckon', 'deadreckon RANGE BEARING FINALHEADING')
    register_action('inventory', 'inventory')
  end
  def navigate(x, y, th)
    puts "stair1 navigate to #{x}, #{y}, #{th}"
    system "#{`rospack find mux`.strip}/switch mux:=velMuxSrv wavefrontVel"
    system "#{`rospack find wavefront_player`.strip}/query_wavefront #{x} #{y} #{th}"
  end
  def opendoor turn_after
    system "#{`rospack find mux`.strip}/switch mux:=velMuxSrv deadReckonVel"
    system "echo \"hello\" |#{`rospack find roscpp_tutorials`.strip}/bin/stdin_query query:=openDoorSrv"
    system "#{`rospack find deadreckon`.strip}/test_deadreckon_service 1 0 #{turn_after}"
  end
  def borgscan
    puts "borgscanning"
    system "#{`rospack find mux`.strip}/switch mux:=finalVelMuxSrv __none"
    sleep 1 # allow the segway to halt
    system 'ssh stair@stair1 "bash -i -c \"cd /home/stair/ros/ros-pkg/unported/vision/sharks/src/lonefleashark && ./patientshark\""'
    system "#{`rospack find mux`.strip}/switch mux:=finalVelMuxSrv activeVel"
    sleep 0.5
  end
  def move_arm location
    puts "moving arm to [#{location}]"
    system('ssh stair@stair1 "bash -i -c \"cd /home/stair/ros/ros-pkg/drivers/robot/katana/nodes/katana_client && ./katana_client ' + location + '\""')
  end
  def deadreckon(range, bearing, final_heading)
    puts "stair1 deadreckon to #{range}, #{bearing}, #{final_heading}"
    system "#{`rospack find mux`.strip}/switch mux:=velMuxSrv deadReckonVel"
    system "#{`rospack find deadreckon`.strip}/test_deadreckon_service #{range} #{bearing} #{final_heading}"
  end
  def inventory
    puts ""
    puts "You are carrying:"
    puts "-----------------"
    puts "segway"
    puts "laser"
    puts "manipulator"
    puts "stereo camera"
    puts "PTZ camera"
    puts "mono camera"
    puts "badass borg laser"
    puts "-----------------"
    puts ""
  end
end
