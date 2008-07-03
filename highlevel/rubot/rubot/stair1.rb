require(`rospack find rubot`.strip + "/rubot/rubot.rb")

class Stair1 < Rubot
  def initialize
    super
    register_action('deadreckon', 'deadreckon RANGE BEARING FINALHEADING')
    register_action('inventory', 'inventory')
  end
  def navigate(x, y, th)
    puts "stair1 navigate to #{x}, #{y}, #{th}"
    system "#{`rospack find mux`.strip}/switch mux:=velMux wavefrontVel"
    system "#{`rospack find wavefront_player`.strip}/query_wavefront #{x} #{y} #{th}"
  end
  def deadreckon(range, bearing, final_heading)
    puts "stair1 deadreckon to #{range}, #{bearing}, #{final_heading}"
    system "#{`rospack find mux`.strip}/switch mux:=velMux deadReckonVel"
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
    puts "-----------------"
    puts ""
  end
end
