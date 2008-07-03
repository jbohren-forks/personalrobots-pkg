require 'irb'
require 'irb/completion'

module IRB
  def IRB.start_session(*args)
    unless $irb
      IRB.setup nil
    end
    workspace = WorkSpace.new(*args)
    $irb = Irb.new(workspace)
    @CONF[:MAIN_CONTEXT] = $irb.context
    trap 'INT' do
      $irb.signal_handle
    end
    catch :IRB_EXIT do
      $irb.eval_input
    end
  end
end

class Rubot
  attr_accessor :actions
  def initialize
    @actions = Array.new
  end
  def register_action name, help
    @actions << [name, help]
  end
  def console
    IRB.start_session(self)
  end
  def show_actions
    puts
    puts "Here are your actions:"
    puts "----------------------"
    actions.each {|a| puts a[1]}
    puts "----------------------"
    nil
  end
end

