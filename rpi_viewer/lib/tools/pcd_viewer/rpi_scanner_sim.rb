
require 'socket'

def write_point( io, azimuth, elevation, distance )
  time = Time.now
  packet = [2,time.utc.to_i,(time.utc.to_f - time.utc.to_i.to_f) * 1000000.0,azimuth,elevation,distance]
  packed_packet = packet.pack('NNNggg')
  io.write( [packed_packet.length + 2].pack('n') + packed_packet )
end

def write_status( io )
  packet = [1,0,10,1,"0.0.1"]
  packed_packet = packet.pack('NNNCa6')
  io.write( [packed_packet.length + 2].pack('n') + packed_packet )
end

def generate_hemisphere
  radius = 1.0

  points = []

  elevation = -1.0 * Math::PI / 2.0
  while (elevation <= (Math::PI / 2.0)) do

    azimuth = -1.0 * Math::PI / 2.0
    while azimuth <= (Math::PI / 2.0) do
      points << [ azimuth, elevation, radius ]
      azimuth += (Math::PI / 100.0)
    end

    elevation += (Math::PI / 100.0)
  end

  return points
end

if ARGV.count < 1
  puts "Please specify server port number"
  exit
end

server = TCPServer.new( ARGV[0].to_i )

points = generate_hemisphere
current_index = 0
puts "Generated hemisphere..."

loop do
  Thread.start( server.accept ) do |client|
    puts "Accepted connection..."

    last_status_time = Time.now
    current_index = 0

    loop do

      azimuth, elevation, distance = points[ current_index ]
      current_index += 1
      current_index = 0 if current_index >= points.size

      begin
        write_point( client, azimuth, elevation, distance )
      rescue StandardError => error
        puts error.message
        puts error.backtrace.join("\n")
        break
      end

      if (last_status_time - Time.now) >= 1.0
        begin
          write_status( client )
        rescue StandardError => error
          puts error.message
          puts error.backtrace.join("\n")
          break
        end

        last_status_time = Time.now
      end

      sleep 0.1

    end
  end
end
