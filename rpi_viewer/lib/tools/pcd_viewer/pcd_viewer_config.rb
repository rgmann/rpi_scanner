
require 'cosmos'

module Cosmos

  class PcdViewerConfig

    attr_reader :point_source

    def initialize( filename )

      @point_source = {}

      parser = ConfigParser.new
      parser.parse_file(filename) do |keyword, params|
        case keyword
        when 'PRIMARY'
          parser.verify_num_parameters(5, 6, "#{keyword} <Target> <Packet> <X/PHI> <Y/THETA> <Z/R> [CART/SPHER]")
          @point_source[:primary] = { target: params[0], packet: params[1] }
          if params[5] && params[5].include?('SPHER')
            @point_source[:primary][:coordinate_system] = :spherical
            @point_source[:primary][:phi] = params[2]
            @point_source[:primary][:theta] = params[3]
            @point_source[:primary][:rho] = params[4]
          else
            @point_source[:primary][:coordinate_system] = :cartesian
            @point_source[:primary][:x] = params[2]
            @point_source[:primary][:y] = params[3]
            @point_source[:primary][:z] = params[4]
          end
        when 'SECONDARY'
          parser.verify_num_parameters(5, 6, "#{keyword} <Target> <Packet> <X/PHI> <Y/THETA> <Z/R> [CART/SPHER]")
          @point_source[:secondary] = { target: params[0], packet: params[1] }
          if params[5] && params[5].include?('SPHER')
            @point_source[:secondary][:coordinate_system] = :spherical
            @point_source[:secondary][:phi] = params[2]
            @point_source[:secondary][:theta] = params[3]
            @point_source[:secondary][:rho] = params[4]
          else
            @point_source[:secondary][:coordinate_system] = :cartesian
            @point_source[:secondary][:x] = params[2]
            @point_source[:secondary][:y] = params[3]
            @point_source[:secondary][:z] = params[4]
          end
        else
          # blank config.lines will have a nil keyword and should not raise an exception
          raise parser.error("Unknown keyword '#{keyword}'") if keyword
        end
      end
    end

  end

end