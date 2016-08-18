
require 'cosmos'
require 'cosmos/tools/opengl_builder/opengl_builder'
require 'cosmos/gui/opengl/gl_shape'
require_relative 'pcd_file'
require_relative 'pcd_viewer_config'
Cosmos.catch_fatal_exception do
  require 'cosmos/script'
  require 'cosmos/gui/qt_tool'
  require 'cosmos/gui/choosers/file_chooser'
  require 'cosmos/gui/choosers/float_chooser'
  require 'cosmos/gui/choosers/string_chooser'
  require 'cosmos/gui/dialogs/splash'
  require 'cosmos/gui/dialogs/progress_dialog'
  require 'cosmos/gui/opengl/gl_viewer'
  require 'cosmos/gui/opengl/stl_shape'
  require 'cosmos/gui/opengl/earth_model'
  require 'cosmos/gui/opengl/moon_model'
  require 'cosmos/gui/dialogs/exception_dialog'
  require 'cosmos/gui/widgets/realtime_button_bar'
  require 'cosmos/tools/opengl_builder/scene_config'
end

include Math

module Cosmos

  class PointShape < Cosmos::GlShape
    def initialize( attrs )
      super( attrs[0], attrs[1], attrs[2] )
      rgb = attrs[3].to_i
      @red   = rgb >> 16
      @green = (rgb >> 8) & 0x000000FF
      @blue  = rgb & 0x000000FF
    end

    def drawshape(viewer)
      GL.Begin GL::POINTS

      GL.Color3ub( @red, @green, @blue )
      GL.Vertex3d( @position[0], @position[1], @position[2] )

      GL.End
    end
  end

  class PointCloudShape < Cosmos::GlShape

    attr_accessor :pcd_file

    def initialize()
      super( 0, 0, 0)

      point_format = PcdFile::PointFormat.new
      point_format.add_dimension( PcdFile::PointFormatDimension.new( 'x', :float, 4, 1 ) )
      point_format.add_dimension( PcdFile::PointFormatDimension.new( 'y', :float, 4, 1 ) )
      point_format.add_dimension( PcdFile::PointFormatDimension.new( 'z', :float, 4, 1 ) )

      @pcd_file = PcdFile.new( point_format )

      @last_start_index = 0

    end
    def drawshape(viewer)
      GL.Begin GL::POINTS

      color = [0,0,0]

      @pcd_file.points[@last_start_index..-1].each do |point|
        GL.Color3ub( color[0], color[1], color[2] )
        GL.Vertex3d( point[0], point[1], point[2] )
      end
      @last_start_index = @pcd_file.points.count - 1 if @last_start_index > 0

      GL.End
    end
  end

  class PcdViewer < QtTool

    def initialize(options)
      super(options) # MUST BE FIRST - All code before super is executed twice in RubyQt Based classes
      Cosmos.load_cosmos_icon("opengl_builder.png")

      @stl_scaling_factor = 1.0
      @bounds = GlBounds.new(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0)
      @scene = GlScene.new
      @previous_selection = nil

      initialize_actions()
      initialize_menus()
      initialize_central_widget()
      complete_initialize()

      @subscription_thread = nil
      @subscription_id = nil
      @cancel_thread = false
      @sleeper = Sleeper.new

      @processed_queue = Queue.new

      @earth_scene = GlScene.new
      @earth_scene.append(EarthModel.new(0.0, 0.0, 0.0))

      @moon_scene = GlScene.new
      @moon_scene.append(MoonModel.new(0.0, 0.0, 0.0))

      statusBar.showMessage("")

      # Handle configuration
      @config_file = options.config_file
      filename = File.join(::Cosmos::USERPATH, %w(config tools pcd_viewer), @config_file)
      if File.exist?(filename)
        @config = PcdViewerConfig.new(filename)
      elsif @config_file != 'pcd_viewer.txt'
        raise "Could not find config file #{filename}"
      end

      @point_cloud = PointCloudShape.new
      @scene.append( @point_cloud )

      @packets = []
      if @config.point_source[:primary]
        @packets << [ @config.point_source[:primary][:target], @config.point_source[:primary][:packet] ]
      end
      if @config.point_source[:secondary]
        @packets << [ @config.point_source[:secondary][:target], @config.point_source[:secondary][:packet] ]
      end

      # Add GUI Update Timeout
      @timer = Qt::Timer.new(self)
      @timer.connect(SIGNAL('timeout()')) { update_gui() }
      @timer.method_missing(:start, 100)

      # puts GL.instance_methods
    end

    def initialize_actions
      super()

      # File Menu
      @file_open = Qt::Action.new(tr('&Open Point Cloud'), self)
      @file_open_key_seq = Qt::KeySequence.new(tr('Ctrl+O'))
      @file_open.shortcut = @file_open_key_seq
      @file_open.statusTip = tr('Open Point Cloud File')
      @file_open.connect(SIGNAL('triggered()')) { file_open() }


      @file_export = Qt::Action.new(tr('&Export Point Cloud'), self)
      @file_export_key_seq = Qt::KeySequence.new(tr('Ctrl+X'))
      @file_export.shortcut = @file_export_key_seq
      @file_export.statusTip = tr('Export Point Cloud to File')
      @file_export.connect(SIGNAL('triggered()')) { file_export() }

      # View Menu
      @view_perspective = Qt::Action.new(tr('&Perspective'), self)
      @view_perspective_key_seq = Qt::KeySequence.new(tr('Ctrl+P'))
      @view_perspective.shortcut = @view_perspective_key_seq
      @view_perspective.statusTip = tr('Perspective View')
      @view_perspective.connect(SIGNAL('triggered()')) { view_perspective() }

      @view_top = Qt::Action.new(tr('&Top'), self)
      @view_top_key_seq = Qt::KeySequence.new(tr('Ctrl+T'))
      @view_top.shortcut = @view_top_key_seq
      @view_top.statusTip = tr('View From Above')
      @view_top.connect(SIGNAL('triggered()')) { view_top() }

      @view_bottom = Qt::Action.new(tr('&Bottom'), self)
      @view_bottom_key_seq = Qt::KeySequence.new(tr('Ctrl+B'))
      @view_bottom.shortcut = @view_bottom_key_seq
      @view_bottom.statusTip = tr('View From Below')
      @view_bottom.connect(SIGNAL('triggered()')) { view_bottom() }

      @view_front = Qt::Action.new(tr('&Front'), self)
      @view_front_key_seq = Qt::KeySequence.new(tr('Ctrl+F'))
      @view_front.shortcut = @view_front_key_seq
      @view_front.statusTip = tr('View From Front')
      @view_front.connect(SIGNAL('triggered()')) { view_front() }

      @view_back = Qt::Action.new(tr('Bac&k'), self)
      @view_back_key_seq = Qt::KeySequence.new(tr('Ctrl+W'))
      @view_back.shortcut = @view_back_key_seq
      @view_back.statusTip = tr('View From Back')
      @view_back.connect(SIGNAL('triggered()')) { view_back() }

      @view_left = Qt::Action.new(tr('&Left'), self)
      @view_left_key_seq = Qt::KeySequence.new(tr('Ctrl+L'))
      @view_left.shortcut = @view_left_key_seq
      @view_left.statusTip = tr('View From Left')
      @view_left.connect(SIGNAL('triggered()')) { view_left() }

      @view_right = Qt::Action.new(tr('&Right'), self)
      @view_right_key_seq = Qt::KeySequence.new(tr('Ctrl+R'))
      @view_right.shortcut = @view_right_key_seq
      @view_right.statusTip = tr('View From Right')
      @view_right.connect(SIGNAL('triggered()')) { view_right() }


      # Capture menu
      @capture_start = Qt::Action.new(tr('Start Live &Capture'), self)
      @capture_start_key_seq = Qt::KeySequence.new(tr('Ctrl+C'))
      @capture_start.shortcut = @capture_start_key_seq
      @capture_start.statusTip = tr('Start live point capture')
      @capture_start.connect(SIGNAL('triggered()')) { handle_start() }

      @capture_stop = Qt::Action.new(tr('&Stop Live Capture'), self)
      @capture_stop_key_seq = Qt::KeySequence.new(tr('Ctrl+S'))
      @capture_stop.shortcut = @capture_stop_key_seq
      @capture_stop.statusTip = tr('Stop live point capture')
      @capture_stop.connect(SIGNAL('triggered()')) { handle_stop() }

      @capture_clear = Qt::Action.new(tr('Capture &Reset'), self)
      @capture_clear_key_seq = Qt::KeySequence.new(tr('Ctrl+R'))
      @capture_clear.shortcut = @capture_clear_key_seq
      @capture_clear.statusTip = tr('Clear captured points')
      @capture_clear.connect(SIGNAL('triggered()')) { handle_stop(); handle_start() }
    end

    def initialize_menus
      # File Menu
      @file_menu = menuBar.addMenu(tr('&File'))
      @file_menu.addAction(@file_open)
      @file_menu.addAction(@file_add_shape)
      @file_menu.addAction(@file_export)
      @file_menu.addSeparator()
      @file_menu.addAction(@exit_action)

      # View Menu
      @view_menu = menuBar.addMenu(tr('&View'))
      @view_menu.addAction(@view_perspective)
      @view_menu.addAction(@view_top)
      @view_menu.addAction(@view_bottom)
      @view_menu.addAction(@view_front)
      @view_menu.addAction(@view_back)
      @view_menu.addAction(@view_left)
      @view_menu.addAction(@view_right)

      # Show Menu
      @capture_menu = menuBar.addMenu(tr('&Capture'))
      @capture_menu.addAction(@capture_start)
      @capture_menu.addAction(@capture_stop)
      @capture_menu.addAction(@capture_clear)

      # Help Menu
      @about_string = "OpenGL Builder is an example application using OpenGL"
      initialize_help_menu()
    end

    def initialize_central_widget
      # Create the central widget
      @central_widget = Qt::Widget.new(self)
      setCentralWidget( @central_widget )

      @top_layout = Qt::VBoxLayout.new(@central_widget)

      # Realtime Button Bar
      @realtime_button_bar = RealtimeButtonBar.new(self)
      @realtime_button_bar.start_callback = method(:handle_start)
      @realtime_button_bar.pause_callback = method(:handle_pause)
      @realtime_button_bar.stop_callback  = method(:handle_stop)
      @realtime_button_bar.state = 'Stopped'
      # @top_layout.addWidget(@realtime_button_bar)
      @top_layout.addWidget(@realtime_button_bar)

      @viewer = GlViewer.new(self)
      @viewer.draw_axis = 15
      @viewer.scene = @scene
      # @viewer.selection_callback = method(:selection_callback)
      @top_layout.addWidget( @viewer )
    end

    def keyPressEvent(event)
      selection = @viewer.selection
      if selection
        color = selection.base_color.clone

        case event.text
        when 'x'
          rotation = selection.rotation_x
          rotation = 0.0 unless rotation
          selection.rotation_x = rotation + 1.0

        when 'X'
          rotation = selection.rotation_x
          rotation = 0.0 unless rotation
          selection.rotation_x = rotation - 1.0

        when 'y'
          rotation = selection.rotation_y
          rotation = 0.0 unless rotation
          selection.rotation_y = rotation + 1.0

        when 'Y'
          rotation = selection.rotation_y
          rotation = 0.0 unless rotation
          selection.rotation_y = rotation - 1.0

        when 'z'
          rotation = selection.rotation_z
          rotation = 0.0 unless rotation
          selection.rotation_z = rotation + 1.0

        when 'Z'
          rotation = selection.rotation_z
          rotation = 0.0 unless rotation
          selection.rotation_z = rotation - 1.0

        when *%w(r R g G b B a A)
          index = 0 # r R
          index = 1 if %w(g G).include?(event.text)
          index = 2 if %w(b B).include?(event.text)
          index = 3 if %w(a A).include?(event.text)

          if %w(R G B A).include?(event.text)
            value = 1.0
            increment = 0.05
          else
            value = 0.0
            # Minimum alpha value shouldn't go to 0 or it disappears
            value = 0.05 if event.text == 'a'
            increment = -0.05
          end

          color[index] += increment
          if value == 1.0
            color[index] = value if color[index] > value
          else
            color[index] = value if color[index] < value
          end
          selection.base_color = color.clone
          selection.color = selection.base_color.clone
          statusBar.showMessage("Color: [#{color[0]} #{color[1]} #{color[2]} #{color[3]}]")

        # TODO: Mouseover tip text is currently not supported in QT OpenGL
        #when 't'
        #  dialog = Qt::Dialog.new(parent, Qt::WindowTitleHint | Qt::WindowSystemMenuHint)
        #  dialog.setWindowTitle('Set Tip Text...')

        #  dialog_layout = Qt::VBoxLayout.new
        #  string_chooser = StringChooser.new(dialog, 'TipText:', selection.tipText.to_s, 60, true)
        #  dialog_layout.addWidget(string_chooser)

        #  button_layout = Qt::HBoxLayout.new
        #  ok = Qt::PushButton.new("Ok")
        #  ok.connect(SIGNAL('clicked()')) { dialog.accept() }
        #  button_layout.addWidget(ok)
        #  cancel = Qt::PushButton.new("Cancel")
        #  cancel.connect(SIGNAL('clicked()')) { dialog.reject() }
        #  button_layout.addWidget(cancel)
        #  dialog_layout.addLayout(button_layout)
        #  dialog.setLayout(dialog_layout)

        #  if dialog.exec == Qt::Dialog::Accepted
        #    tipText = string_chooser.string
        #    if tipText.to_s.empty?
        #      selection.tipText = nil
        #    else
        #      selection.tipText = tipText
        #    end
        #  end
        #  dialog.dispose

        end # case event.text

      end # if selection

    end # def keyPressEvent

    def handle_reset
      System.telemetry.reset
    end

    def handle_start
      # if windowTitle() != 'Data Viewer'
      #   # Clear Title
      #   setWindowTitle('Data Viewer')

      #   # Return to default configuration
      #   # Show splash during possible reconfiguration
      #   Splash.execute(self) do |splash|
      #     ConfigParser.splash = splash
      #     System.load_configuration
      #     ConfigParser.splash = nil
      #   end

      #   # Reset Components
      #   handle_reset()
      # end

      # Restart GUI Updates if necessary
      if @pause
        @pause = false
        @timer.method_missing(:start, 100)
      end

      # Restart Subscription Thread if necessary
      start_subscription_thread()
    end

    def handle_pause
      @pause = true
      if @subscription_thread
        @realtime_button_bar.state = 'Paused'
      end
    end

    def handle_stop
      # Shutdown packet subscription thread
      kill_subscription_thread()

      # Unsubscribe from subscription
      if @subscription_id
        begin
          unsubscribe_packet_data(@subscription_id)
        rescue DRb::DRbConnError
          # Do Nothing
        end
      end

      # Restart GUI Updates if necessary
      if @pause
        @pause = false
        @timer.method_missing(:start, 100)
      end

      @realtime_button_bar.state = 'Stopped'
    end

    def start_subscription_thread
      unless @subscription_thread
        # Start Thread to Gather Events
        @subscription_thread = Thread.new do

          @cancel_thread = false
          @sleeper = Sleeper.new

          if !@packets.empty?
            begin
              while true
                break if @cancel_thread

                begin
                  @subscription_id = subscribe_packet_data( @packets, 10000 )
                  break if @cancel_thread

                  if @pause
                    Qt.execute_in_main_thread(true) { @realtime_button_bar.state = 'Paused' }
                  else
                    Qt.execute_in_main_thread(true) { @realtime_button_bar.state = 'Running' }
                  end
                  Qt.execute_in_main_thread(true) { statusBar.showMessage("Connected to Command and Telemetry Server: #{Time.now.formatted}") }
                rescue DRb::DRbConnError
                  break if @cancel_thread
                  Qt.execute_in_main_thread(true) do
                    @realtime_button_bar.state = 'Connecting'
                    statusBar.showMessage(tr("Error Connecting to Command and Telemetry Server"))
                  end
                  break if @sleeper.sleep(1)
                  break if @cancel_thread
                  retry
                end

                while true
                  break if @cancel_thread
                  begin
                    # Get a subscribed to packet
                    packet_data, target_name, packet_name, received_time, received_count = get_packet_data(@subscription_id)

                    # Put packet data into its packet
                    packet = System.telemetry.packet(target_name, packet_name)
                    packet.buffer = packet_data
                    packet.received_time = received_time
                    packet.received_count = received_count

                    point_source_config = @config.point_source[:primary]
                    if point_source_config[:coordinate_system] == :cartesian
                      point = [
                        packet.read(point_source_config[:x]),
                        packet.read(point_source_config[:y]),
                        packet.read(point_source_config[:z])]
                    else
                      point = spherical_to_cartesian(
                        packet.read(point_source_config[:phi]),
                        packet.read(point_source_config[:theta]),
                        packet.read(point_source_config[:rho]) * 1000.0)
                      # puts point
                    end

                    @point_cloud.pcd_file.add_point( point )
                    @processed_queue << point

                  rescue DRb::DRbConnError
                    break if @cancel_thread
                    Qt.execute_in_main_thread(true) { statusBar.showMessage(tr("Error Connecting to Command and Telemetry Server")) }
                    break # Let outer loop resubscribe
                  rescue RuntimeError => error
                    raise error unless error.message =~ /queue/
                    break if @cancel_thread
                    Qt.execute_in_main_thread(true) { statusBar.showMessage(tr("Connection Dropped by Command and Telemetry Server: #{Time.now.formatted}")) }
                    break # Let outer loop resubscribe
                  end
                end
              end
            rescue Exception => error
              break if @cancel_thread
              Qt.execute_in_main_thread(true) { || ExceptionDialog.new(self, error, "Data Viewer : Subscription Thread")}
            end
          end # if !@packets.empty
        end

      else
        Qt.execute_in_main_thread(true) { @realtime_button_bar.state = 'Running' }
      end
    end

    def kill_subscription_thread
      Cosmos.kill_thread(self, @subscription_thread)
      @subscription_thread = nil
    end

    def update_gui
      begin
        if @pause
          @timer.method_missing(:stop)
        else
          Qt.execute_in_main_thread(true) do
            point = @processed_queue.pop(true)

            # @scene.append PointShape.new( point )
            @scene.draw( @viewer )
            @viewer.update
          end
        end
      rescue ThreadError
        # Nothing to do
      end
    end

    def spherical_to_cartesian( phi, theta, rho )
      z = rho * Math.sin( theta ) * Math.cos( phi )
      x = rho * Math.sin( theta ) * Math.sin( phi )
      y = rho * Math.cos( theta )
      return [ x, y, z ]
    end

    def file_open
      filename = Qt::FileDialog.getOpenFileName(self, "Open Scene File:", File.join(USERPATH, 'config', 'tools', 'opengl_builder'), "PCD Files (*.pcd);;All Files (*)")
      if !filename.to_s.empty?
        begin
          @point_cloud.pcd_file = PcdFile.parse!( filename )
          @viewer.update

          statusBar.showMessage "Loaded #{filename}"
        rescue Exception => error
          ExceptionDialog.new( self, error, "Error Loading Scene", false )
        end
      end
    end

    def file_export
      filename = Qt::FileDialog.getSaveFileName(self, "Export Point Cloud to File", File.join(USERPATH, 'config', 'tools', 'opengl_builder', 'scene.txt'), "Scene Files (*.txt);;All Files (*)")
      if !filename.to_s.empty?
        @point_cloud.pcd_file.write( filename )
      end
    end

    def view_perspective
      @viewer.orientation = Quaternion.new([1.0, 1.0, 0.0], 0.8)
    end

    def view_top
      @viewer.orientation = Quaternion.new([1.0, 0.0, 0.0], PI/2)
    end

    def view_bottom
      @viewer.orientation = Quaternion.new([1.0, 0.0, 0.0], -PI/2)
    end

    def view_front
      @viewer.orientation = Quaternion.new([1.0, 0.0, 0.0], 0.0)
    end

    def view_back
      @viewer.orientation = Quaternion.new([1.0, 0.0, 0.0], PI)
    end

    def view_left
      @viewer.orientation = Quaternion.new([0.0, 1.0, 0.0], PI/2)
    end

    def view_right
      @viewer.orientation = Quaternion.new([0.0, 1.0, 0.0], -PI/2)
    end

    def show_scene
      @viewer.scene = @scene
    end

    def show_earth
      @viewer.scene = @earth_scene
    end

    def show_moon
      @viewer.scene = @moon_scene
    end

    # Runs the application
    def self.run (option_parser = nil, options = nil)
      Cosmos.catch_fatal_exception do
        unless option_parser and options
          option_parser, options = create_default_options()
          options.title = "Point Cloud Viewer"
          options.config_file = "pcd_viewer.txt"
        end
        super(option_parser, options)
      end
    end

  end

end

