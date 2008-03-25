require 'rosrb/serializable.rb'

class FlowpyImageString < Flow
  def initialize name
    super()
    @name = name
    @datatype = "pyImageViewer/pyImageString"
    @imageString = ''
    @imageFormat = ''
    @height = 0
    @width = 0
  end
  attr_accessor :imageString
  attr_accessor :imageFormat
  attr_accessor :height
  attr_accessor :width
  def serialize
    init_serialization
    serialize_string(@imageString)
    serialize_string(@imageFormat)
    serialize_uint32(@height)
    serialize_uint32(@width)
    @serialization
  end
  def deserialize s
    init_deserialization s
    @imageString = deserialize_string
    @imageFormat = deserialize_string
    @height = deserialize_uint32
    @width = deserialize_uint32
  end
  def print indent=0
    istr = ''
    indent.times{ istr << '  ' }
    puts istr + "imageString = #{@imageString}"
    puts istr + "imageFormat = #{@imageFormat}"
    puts istr + "height = #{@height}"
    puts istr + "width = #{@width}"
  end
end
