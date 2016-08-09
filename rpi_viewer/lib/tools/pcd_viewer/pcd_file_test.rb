require './pcd_file.rb'

file = PcdFile.parse!( 'pcd_test.pcd' )

puts file.version
puts file.points.count
