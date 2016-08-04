
class PcdFile

  class PointFormatDimension
    attr_accessor :name
    attr_accessor :size
    attr_accessor :type
    attr_accessor :count

    def initialize(name,type=:float,size=4,count=1)
      @name = name
      @type = type
      @size = size
      @count = count
    end

    def set_size( size )
      return false unless [1,2,4,8].include? size.to_i
      @size = size.to_i
      return true
    end

    def set_type( type )
      valid_types = {'i' => :signed, 'u' => :unsigned, 'f' => :float }
      return false unless valid_types.keys.include? type.downcase
      @type = valid_types[type]
      return true
    end

    def set_count( count )
      @count = count
      return true
    end
  end

  class PointFormat
    attr_accessor :dimensions
    def initialize
      @dimensions = []
    end
    def add_dimension(dimension)
      @dimensions << dimension
    end

    def <<( dimension )
      add_dimension( dimension )
    end

    def [](index)
      return @dimensions[index]
    end
  end

  class Field

    attr_reader :name
    attr_accessor :found

    def initialize( name, arg_limits, required = true )
      @found = false
      @name = name
      @arg_limits = arg_limits
      @required = required
    end

    def process( file, arguments )
      if arguments.count < @arg_limits[:min] || arguments.count > @arg_limits[:max]
        raise "field #{@name} expected [#{@arg_limits[:min]},#{@arg_limits[:max]}] arguments; found #{arguments.count}"
      end
      process_args( file, arguments )
    end

    def process_args( file, arguments )
      # do nothing
    end
  end

  class VersionField < Field
    def initialize
      super( 'VERSION', { min: 1, max: 1 } )
    end
    def process_args(file,args)
      file.version = args.first
    end
  end

  class FormatField < Field
    def initialize
      super( 'FIELDS', { min: 1, max: 6 } )
    end
    def process_args(file,args)
      args.each do |field|
        file.point_format << PointFormatDimension.new(field)
      end
    end
  end

  class SizeField < Field
    def initialize
      super( 'SIZE', { min: 1, max: 6 } )
    end
    def process_args(file,args)
      if args.count != file.point_format.dimensions.count
        raise "field size count does not equal field count"
      else
        args.each_with_index do |arg,index|
          unless file.point_format[index].set_size( arg )
            raise ""
          end
        end
      end
    end
  end

  class TypeField < Field
    def initialize
      super( 'TYPE', { min: 1, max: 6 } )
    end
    def process_args(file,args)
      if args.count != file.point_format.dimensions.count
        raise "field type count does not equal field count"
      else
        args.each_with_index do |arg,index|
          unless file.point_format[index].set_type( arg )
            raise ""
          end
        end
      end
    end
  end

  class CountField < Field
    def initialize
      super( 'COUNT', { min: 1, max: 6 } )
    end
    def process_args(file,args)
      if args.count != file.point_format.dimensions.count
        raise "field type count does not equal field count"
      else
        args.each_with_index do |arg,index|
          unless file.point_format[index].set_count( arg )
            raise ""
          end
        end
      end
    end
  end

  class WidthField < Field
    def initialize
      super( 'WIDTH', { min: 1, max: 1 } )
    end
    def process_args( file, args )
      if args[0] =~ /\d+/
        file.width = args[0].to_i
      else
        raise ""
      end
    end
  end

  class HeightField < Field
    def initialize
      super( 'HEIGHT', { min: 1, max: 1 } )
    end
    def process_args(file,args)
      if args[0] =~ /\d+/
        file.height = args[0].to_i
      else
        raise ""
      end
    end
  end

  class ViewPointField < Field
    def initialize
      super( 'VIEWPOINT', { min: 7, max: 7 } )
    end
    def process_args(file,args)
      file.viewpoint[:transform][:x] = args[0]
      file.viewpoint[:transform][:y] = args[1]
      file.viewpoint[:transform][:z] = args[2]

      file.viewpoint[:quaternion][:w] = args[3]
      file.viewpoint[:quaternion][:x] = args[4]
      file.viewpoint[:quaternion][:y] = args[5]
      file.viewpoint[:quaternion][:z] = args[6]
    end
  end

  class PointsField < Field
    def initialize
      super( 'POINTS', {min:1, max:1}, false )
    end
  end

  class DataField < Field
    def initialize
      super( 'DATA' , {min:1, max:1} )
    end
    def process_args( file, args )
      case args.first
      when 'ascii'
        file.format = :ascii
      when 'binary'
        file.format = :binary
      else
        raise "invalid DATA format: #{field_attrs.first}"
      end
    end
  end

  attr_accessor :version
  attr_accessor :format
  attr_accessor :point_format
  attr_accessor :width
  attr_accessor :height
  attr_accessor :viewpoint
  attr_accessor :points

  attr_accessor :fields

  def initialize( point_format = nil )
    @fields = []
    @fields << VersionField.new
    @fields << FormatField.new
    @fields << SizeField.new
    @fields << CountField.new
    @fields << TypeField.new
    @fields << WidthField.new
    @fields << HeightField.new
    @fields << ViewPointField.new
    @fields << PointsField.new
    @fields << DataField.new

    @version = nil
    @points = []

    if point_format
      @point_format = point_format
    else
      @point_format = PointFormat.new
    end

    @viewpoint = {
      transform:  { x: 0, y: 0, z: 0 },
      quaternion: { w: 0, x: 0, y: 0, z: 0 } }
  end

  def valid?
  end

  def self.parse!( filepath )

    file = PcdFile.new()
    
    File.open( filepath, 'r' ) do |io|

      header_complete = false
      while header_complete == false do

        line = io.gets
        break if line.nil?

        # Discard comments
        next if line =~ /\s*#/
        next if line.strip.empty?

        tokens = line.split(/\s/)
        tokens.map! { |item| item.strip }

        field_name  = tokens.first
        arguments = tokens[1..-1]

        field = file.fields.find { |a_field| a_field.name == field_name }

        if field
          field.process( file, arguments )
          field.found = true
        else
          raise "unknown field #{field_name}"
        end

        unset_fields = file.fields.find_all { |field| !field.found }
        header_complete = unset_fields.empty?

      end

      if file.format == :ascii
        file.points = read_points_ascii( io, file.point_format )
      elsif file.format == :binary
        file.points = read_points_binary( io, file.point_format )
      end

    end

    return file

  end

  def self.parse( filepath )
    begin
      return parse!( filepath )
    rescue StandardError => error
      return nil
    end
  end

  def self.read_points_ascii( io, point_format )
    points = []
    while (line = io.gets) do
      tokens = line.split(/\s/)
      if tokens.count != point_format.dimensions.count
        raise "malformed point: invalid parameter count"
      end

      point = []
      point_format.dimensions.each_with_index do |dim,index|
        token = tokens[index].strip
        if dim.type == :float
          if token =~ /[\+-]?\d*\.?\d+([Ee][\+-]?\d+)?/
            point << token.to_f
          else
            raise ""
          end
        elsif dim.type == :unsigned
          if token =~ /\+?\d+/
            point << token.to_i
          else
            raise ""
          end
        else
          if token =~ /[\+-]?\d+/
            point << token.to_f
          else
            raise ""
          end
        end
      end
      points << point
    end
    return points
  end

  def add_point( point )
    if (point.class == Array) && (point.count == @point_format.dimensions.count)
      @points << point
    end
  end

  def write( filepath )
    success = valid?

    File.open( filepath, 'w+' ) do |io|

      io.puts "VERSION #{@version}"

      fields = []
      sizes  = []
      types  = []
      counts = []
      @point_format.dimensions.each do |dim|
        fields << dim.name
        sizes  << dim.size
        types  << { signed: 'I', unsigned: 'U', float: 'F' }[dim.type]
        counts << dim.count
      end
      io.puts "FIELDS #{fields.join(' ')}"
      io.puts "SIZE #{sizes.join(' ')}"
      io.puts "TYPE #{types.join(' ')}"
      io.puts "COUNT #{counts.join(' ')}"

      io.puts "WIDTH #{@width}"
      io.puts "HEIGHT #{@height}"
      io.puts "VIEWPOINT #{@viewpoint[:transform].values.join(' ')} #{@viewpoint[:quaternion].values.join(' ')}"
      io.puts "POINTS #{@points.count}"
      io.puts "DATA #{@format.to_s}"

      if @format == :ascii
        write_points_ascii( io )
      elsif @format == :binary
        write_points_binary( io )
      end

    end

    return success
  end


  def write_points_ascii( io )
    @points.each do |point|
      io.puts point.join(' ')
    end
  end

  def write_points_binary( io )
  end

end

