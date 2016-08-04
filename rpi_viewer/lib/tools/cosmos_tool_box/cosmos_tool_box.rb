
require 'cosmos'
require 'json'
require 'net/http'
Cosmos.catch_fatal_exception do
  require 'cosmos/script'
  require 'cosmos/config/config_parser'
  require 'cosmos/gui/qt_tool'
  require 'cosmos/gui/dialogs/splash'
  require 'cosmos/gui/dialogs/progress_dialog'
  require 'cosmos/gui/dialogs/exception_dialog'
  require 'cosmos/gui/dialogs/packet_log_dialog'
  require 'cosmos/gui/dialogs/find_replace_dialog'
  require 'cosmos/gui/widgets/realtime_button_bar'
  require 'cosmos/tools/data_viewer/data_viewer_component'
  require 'cosmos/tools/data_viewer/dump_component'
end

module Cosmos

  module RubygemsApi

    module DotOrgApi
      def self.api_uri( method, params={} )
        url = "https://www.rubygems.org/api/v1/#{method}"

        param_tokens = []
        params.each { |name,value| param_tokens << [name.to_s,value.to_s].join('=') }
        if !param_tokens.empty?
          url += "?#{param_tokens.join(',')}"
        end
        
        return URI(url)
      end
      def self.search_gems( part_of_name )
        gem_info_list = []
        begin
          response_json = Net::HTTP.get( self.api_uri( 'search.json', { query: part_of_name } ) )
          gem_info_list = JSON.parse(response_json)
        rescue StandardError => err
          puts err.message
          puts err.backtrace.join "\n"
        end

        return gem_info_list
      end

      def self.get_gem_by_name( name )
        gem_info = nil
        uri = self.api_uri( "gems/#{name}.json" )
        begin
          response_json = Net::HTTP.get( uri )
          gem_info = JSON.parse(response_json)
        rescue StandardError => err
          puts err.message
          puts err.backtrace.join("\n")
          raise err
        end

        return gem_info
      end

      def self.get_gems_by_name( gem_names )
        gem_info_list = []

        begin
          gem_names.each do |gem_name|
            gem_info_list << self.get_gem_by_name( gem_name )
          end
        rescue StandardError => err
        end

        return gem_info_list
      end

    end

  end

  class Gemfile

    def initialize( path )
    end

    def add_gem( gem_name, version_opts={} )
      version = {
        version: :head,
        rel: :equal
      }

      existing_gem = @gems.find {|a_gem| a_gem[:name] == gem_name }
      if existing_gem
        existing_gem[:version] = version
      else
        new_gem = { name: gem_name, version: version }
      end

    end

    def remove_gem( gem_name )
      @gems.drop do |a_gem|
        a_Gem[:name] == gem_name
      end
    end

    protected

    SECTION_START = /#\s*BEGIN\s+COSMOS_TOOL_GEM_LIST/
    SECTION_END   = /#\s*END\s+COSMOS_TOOL_GEM_LIST/

    def read( path )

      # The Gemfile must be annotated with BEGIN COSMOS_TOOL_GEM_LIST and
      # END COSMOS_TOOL_GEM_LIST.

      begin
        File.open( File.join( path, 'Gemfile' ), 'r' ) do |io|
          gemfile_text = io.read

          section_start = gemfile_text.index( SECTION_START )
          section_end   = gemfile_text.index( SECTION_END )

          if section_start && section_end
            text_of_interest = gemfile_text[section_start,(section_end-section_start)]

            
          end

        end
      rescue StandardError => error
      end
    end

  end

  module GemspecManager
    def self.add_gem( gem_name, version, backup )
    end
    def self.remove_gem( gem_name, backup )
    end
    def self.list
      gem_list_text = ''
      Open3.popen3('bundle exec gem list --local') do |stdin,stdout,sterr,wait_thr|
        stdout_thread = Thread.new do
          begin
            while (bytes = stdout.gets) do
              gem_list_text << bytes
            end
          rescue Exception => e
          end
        end
        stdout_thread.join
      end

      gem_list = {}
      gem_list_text.split(/\r?\n/).each do |gem_line|
        gem_name, gem_version = gem_line.split(/\s/)
        gem_list[gem_name] = gem_version.scan(/\(([0-9\.]+)\)/)[0][0]
      end

      return gem_list
    end
  end


  class ToolViewContainer < Qt::Widget
    def initialize(parent)
      super( parent )
      @active_tool = nil
    end
    def initialize_gui
      @top_layout = Qt::HBoxLayout.new(self)
      @grid_layout = Qt::GridLayout.new
      @top_layout.addLayout( @grid_layout )

      @grid_layout.addWidget( Qt::Label.new('Name'), 0, 0 )
      @tool_name = Qt::Label.new('')
      @grid_layout.addWidget( @tool_name, 0, 1 )

      @grid_layout.addWidget( Qt::Label.new('Version'), 1, 0 )
      @tool_version = Qt::Label.new('')
      @grid_layout.addWidget( @tool_version, 1, 1 )

      @grid_layout.addWidget( Qt::Label.new('Info'), 2, 0 )
      @tool_info = Qt::Label.new('')
      @grid_layout.addWidget( @tool_info, 2, 1 )

      @install_button = Qt::PushButton.new('Install')
      @grid_layout.addWidget( @install_button, 3, 0, 1, 2 )

      setLayout( @top_layout )
    end
    def set_active_tool(tool_info)
      @active_tool_info = tool_info
      @tool_name.setText(tool_info['name'])
      @tool_version.setText(tool_info['version'])
      @tool_info.setText(tool_info['info'])
    end
  end

  class ToolListWidget < Qt::ListWidget
    def initialize(parent)
      super(parent)
    end
    def initialize_gui
      @top_layout = Qt::VBoxLayout.new
      setLayout(@top_widget)
    end
    def set_tool_list(tool_list)
      @tool_list = tool_list
      @tool_list.each do |tool|
        # tool_button = Qt::PushButton.new(tool['name'])
        # stylesheet = "margin: 0px; padding:4px; text-align:center; " \
        #   "font-family:Arial; " \
        #   "font-size:12px"
        # tool_button.setStyleSheet(stylesheet)
        # tool_button.setFixedSize(120,74)
        # tool_button.connect(SIGNAL('clicked()')) { @right_widget.set_active_tool(tool) }
        # @top_layout.addWidget( tool_button )
        Qt::ListWidgetItem.new(tool['name'],self)
      end
      # @right_widget.set_active_tool( tool_list.first )
    end
  end

  class ToolListTab < Qt::Widget

    slots 'select_tool(QListWidgetItem*)'

    attr_reader :tab_name
    def initialize(parent,tab_name)
      super(parent)
      @tab_name = tab_name
    end
    def initialize_gui
      @top_widget = Qt::Splitter.new(Qt::Horizontal,self)
      @top_layout = Qt::HBoxLayout.new(@top_widget)
      # @left_widget.setLayout(@left_frame)

      # @left_widget = Qt::Widget.new(self)
      @left_widget = ToolListWidget.new(self)
      connect(@left_widget,SIGNAL('itemClicked(QListWidgetItem*)'),self,SLOT('select_tool(QListWidgetItem*)'))
      @left_frame = Qt::VBoxLayout.new
      @left_widget.setLayout(@left_frame)
      # @left_frame = ToolList.new(self)
      # @left_frame.initialize_gui
      @top_layout.addWidget( @left_widget )

      # @right_widget = Qt::Widget.new(self)
      @right_widget = ToolViewContainer.new(self)
      @right_widget.initialize_gui
      @right_frame = Qt::VBoxLayout.new
      @right_widget.setLayout(@right_frame)
      @top_layout.addWidget( @right_widget )

      setLayout( @top_layout )
    end

    def select_tool( item )
      selected_tool = @tool_list.find do |tool|
        tool['name'] == item.text
      end
      if selected_tool
        @right_widget.set_active_tool( selected_tool )
      end
    end

    def set_tool_list(tool_list)
      @tool_list = tool_list
      @left_widget.set_tool_list( tool_list )
    end
  end

  class CosmosToolBox < QtTool
    slots 'context_menu(const QPoint&)'
    slots 'handle_tab_change(int)'

    def initialize(options)
      super(options) # MUST BE FIRST - All code before super is executed twice in RubyQt Based classes
      Cosmos.load_cosmos_icon("data_viewer.png")

      initialize_actions()
      initialize_menus()
      initialize_central_widget()
      complete_initialize()

      # Initialize instance variables
      @auto_start = false
      @pause = false
      @log_file_directory = System.paths['LOGS']
      @tlm_log_system = nil
      @subscription_thread = nil
      @subscription_id = nil
      @packet_log_reader = System.default_packet_log_reader.new
      @time_start = nil
      @time_end = nil
      @log_filenames = []
      @cancel_thread = false
      @sleeper = Sleeper.new

      # Process config file and create a tab for each data viewer component
      @tab_mutex = Mutex.new
      @tabs = []
      @packets = []
      @packet_to_components_mapping = {}
      # @config_filename = File.join(Cosmos::USERPATH, 'config', 'tools', 'data_viewer', options.config_file)
      # process_config()

      Qt.execute_in_main_thread(true) do
        @tabs << ToolListTab.new(self,'All Tools')
        @tabs << ToolListTab.new(self,'Installed Tools')
        @tabs.each do |tab|
          tab.initialize_gui
          @tab_book.addTab( tab, tab.tab_name )
        end
      end

      # Query a remote list of COSMOS tools from rubygems.org
      @tool_list = RubygemsApi.search_gems( 'cosmos-' )
      @tool_list.each do |tool|
        append_status( "Found remote tool \"#{tool['name']}\" @ v#{tool['version']}")
      end

      # Find COSMOS tool gems in the gemspec and retrieve there information
      # from rubygems.org.
      installed_tools.each do |name,version|
        append_status( "Found local tool \"#{name}\" @ v#{version}")

        tool_gem_info = @tool_list.find { |tool_info| tool_info['name'] == name }
        if tool_gem_info
          tool_gem_info['installed'] = { name: name, version: version }
        else
          begin
            tool_gem_info = RubygemsApi.get_gem_by_name( name )
            tool_gem_info['installed'] = { name: name, version: version }
            @tool_list << tool_gem_info
          rescue StandardError => error
            append_status(error.message)
          end
        end
      end

      # puts @tool_list
      Qt.execute_in_main_thread(true) do
        @tabs.first.set_tool_list( @tool_list )

        installed_tools = @tool_list.find_all { |tool_info| tool_info['installed'].nil? == false }
        @tabs.last.set_tool_list( installed_tools )
      end

      # Add GUI Update Timeout
      # @timer = Qt::Timer.new(self)
      # @timer.connect(SIGNAL('timeout()')) { update_gui() }
      # @timer.method_missing(:start, 100)
    end

    def initialize_actions
      super()

      # File Menu Actions
      @open_log = Qt::Action.new(tr('&Open Log File'), self)
      @open_log_keyseq = Qt::KeySequence.new(tr('Ctrl+O'))
      @open_log.shortcut  = @open_log_keyseq
      @open_log.statusTip = tr('Open telemetry log file for processing')
      @open_log.connect(SIGNAL('triggered()')) { handle_open_log_file() }

      @handle_reset = Qt::Action.new(tr('&Reset'), self)
      @handle_reset_keyseq = Qt::KeySequence.new(tr('Ctrl+R'))
      @handle_reset.shortcut  = @handle_reset_keyseq
      @handle_reset.statusTip = tr('Reset Components')
      @handle_reset.connect(SIGNAL('triggered()')) { handle_reset() }

      # Search Actions
      @search_find = Qt::Action.new(Cosmos.get_icon('search.png'), tr('&Find'), self)
      @search_find_keyseq = Qt::KeySequence.new(tr('Ctrl+F'))
      @search_find.shortcut  = @search_find_keyseq
      @search_find.statusTip = tr('Find text')
      @search_find.connect(SIGNAL('triggered()')) do
        FindReplaceDialog.show_find(self)
      end

      @search_find_next = Qt::Action.new(tr('Find &Next'), self)
      @search_find_next_keyseq = Qt::KeySequence.new(tr('F3'))
      @search_find_next.shortcut  = @search_find_next_keyseq
      @search_find_next.statusTip = tr('Find next instance')
      @search_find_next.connect(SIGNAL('triggered()')) do
        FindReplaceDialog.find_next(self)
      end

      @search_find_previous = Qt::Action.new(tr('Find &Previous'), self)
      @search_find_previous_keyseq = Qt::KeySequence.new(tr('Shift+F3'))
      @search_find_previous.shortcut  = @search_find_previous_keyseq
      @search_find_previous.statusTip = tr('Find previous instance')
      @search_find_previous.connect(SIGNAL('triggered()')) do
        FindReplaceDialog.find_previous(self)
      end

      # Tab Menu Actions
      @delete_tab = Qt::Action.new(Cosmos.get_icon('delete_tab.png'), tr('&Delete Tab'), self)
      @delete_tab.statusTip = tr('Delete active tab')
      @delete_tab.connect(SIGNAL('triggered()')) { on_tab_delete() }
    end

    def initialize_menus
      # File Menu
      file_menu = menuBar.addMenu(tr('&File'))
      file_menu.addAction(@open_log)
      file_menu.addAction(@handle_reset)
      file_menu.addSeparator()
      file_menu.addAction(@exit_action)

      # Tab Menu
      @tab_menu = menuBar.addMenu(tr('&Tab'))
      @tab_menu.addAction(@delete_tab)
      @tab_menu.addSeparator()
      @tab_menu_actions = []

      # Search Menu
      view_menu = menuBar.addMenu(tr('&Search'))
      view_menu.addAction(@search_find)
      view_menu.addAction(@search_find_next)
      view_menu.addAction(@search_find_previous)

      # Help Menu
      @about_string = "Data Viewer is designed to allow users view data that is not easily displayed on telemetry screens."
      initialize_help_menu()
    end

    def initialize_central_widget
      # Create the central widget
      @central_widget = Qt::Widget.new
      setCentralWidget(@central_widget)

      # Create the top level vertical layout
      @top_layout = Qt::VBoxLayout.new(@central_widget)

      # Realtime Button Bar
      # @realtime_button_bar = RealtimeButtonBar.new(self)
      # @realtime_button_bar.start_callback = method(:handle_start)
      # @realtime_button_bar.pause_callback = method(:handle_pause)
      # @realtime_button_bar.stop_callback  = method(:handle_stop)
      # @realtime_button_bar.state = 'Stopped'
      # @top_layout.addWidget(@realtime_button_bar)
      @refresh_button = Qt::PushButton.new('Refresh')
      connect(@refresh_button,SIGNAL('clicked()')) { handle_refresh() }
      @top_layout.addWidget(@refresh_button)

      # @list_button = Qt::PushButton.new('List')
      # connect(@list_button,SIGNAL('clicked()')) { list_installed() }
      # @top_layout.addWidget(@list_button)

      # Create tab book
      @tab_book = Qt::TabWidget.new
      @tab_book.setContextMenuPolicy(Qt::CustomContextMenu)
      connect(@tab_book, SIGNAL('currentChanged(int)'), self, SLOT('handle_tab_change(int)'))
      # connect(@tab_book, SIGNAL('customContextMenuRequested(const QPoint&)'), self, SLOT('context_menu(const QPoint&)'))
      @top_layout.addWidget(@tab_book)

      # Text status box
      @status = ''
      @status_box = Qt::TextEdit.new
      @status_box.setReadOnly( true )
      @top_layout.addWidget(@status_box)
    end

    def append_status( message )
      @status << "#{message}<br/>"
      Qt.execute_in_main_thread(true) do
        @status_box.setHtml( @status )
      end
    end

    def handle_refresh
      uri = "https://www.rubygems.org/api/v1/search.json?query=bbc-cosmos-"

      begin
        response_json = Net::HTTP.get(URI(uri))
        response = JSON.parse(response_json)
        @tabs.each do |tab|
          tab.set_tool_list( response )
        end
      rescue StandardError => err
        puts err.message
        puts err.backtrace.join "\n"
      end
    end

    def current_tab
      @tab_mutex.synchronize do
        yield @tabs[@tab_book.currentIndex()]
      end
    end

    # Called by the FindReplaceDialog to get the text to search
    def search_text
      current_tab do |component|
        component.text
      end
    end

    def handle_tab_change(index)
      # # Remove existing actions
      # @tab_menu_actions.each do |action|
      #   @tab_menu.removeAction(action)
      #   action.dispose
      # end
      # @tab_menu_actions = []

      # # Add new actions
      # unless @components.empty?
      #   @component_mutex.synchronize do
      #     component = @components[index]
      #     component.packets.each do |target_name, packet_name|
      #       action = Qt::Action.new("#{target_name} #{packet_name}", @tab_menu)
      #       action.setCheckable(true)
      #       key = target_name + ' ' + packet_name
      #       components = @packet_to_components_mapping[key]
      #       if components.include?(component)
      #         action.setChecked(true)
      #       else
      #         action.setChecked(false)
      #       end
      #       action.connect(SIGNAL('triggered()')) { enable_disable_component_packet(index, target_name, packet_name, action.checked?) }
      #       @tab_menu.addAction(action)
      #       @tab_menu_actions << action
      #     end
      #   end
      # end
    end

    def installed_tools
      return GemspecManager.list
    end

    def context_menu(point)
      index = 0
      (0..@tab_book.tabBar.count).each do |i|
        index = i if @tab_book.tabBar.tabRect(i).contains(point)
      end

      return if (index == @tab_book.tabBar.count)

      # Bring the right clicked tab to the front
      @tab_book.setCurrentIndex(index)
      @tab_menu.exec(@tab_book.mapToGlobal(point))
    end

    def cancel_callback(progress_dialog = nil)
      @cancel_progress = true
      return true, false
    end

    def update_gui
      Qt.execute_in_main_thread(true) do
        @component_mutex.synchronize do
          @tabs.each do |tab|
            tab.update_gui
          end
        end
      end
    end

    def closeEvent(event)
      # Accept closure
      super(event)
    end

    def process_config
      # ensure the file exists
      unless test ?f, @config_filename
        raise "Configuration File Does not Exist: #{@config_filename}"
      end

      parser = ConfigParser.new
      parser.parse_file(@config_filename) do |keyword, params|
        case keyword

        when 'AUTO_START'
          usage = "#{keyword}"
          parser.verify_num_parameters(0, 0, usage)
          @auto_start = true

        when 'COMPONENT'
          usage = "#{keyword} <tab name> <component class filename> <component specific options...>"
          parser.verify_num_parameters(2, nil, usage)
          component_class = Cosmos.require_class(params[1])
          if params.length >= 3
            @components << component_class.new(self, params[0], *params[2..-1])
          else
            @components << component_class.new(self, params[0])
          end

        when 'PACKET'
          usage = "#{keyword} <target_name> <packet_name>"
          parser.verify_num_parameters(2, 2, usage)
          target_name = params[0].upcase
          packet_name = params[1].upcase
          @packets << [target_name, packet_name]
          @components[-1].add_packet(target_name, packet_name)
          index = target_name + ' ' + packet_name
          @packet_to_components_mapping[index] ||= []
          @packet_to_components_mapping[index] << @components[-1]

        # Unknown keyword
        else
          raise "Unhandled keyword: #{keyword}" if keyword
        end
      end
    end

    

    #############################
    # Class methods
    #############################

    def self.run(option_parser = nil, options = nil)
      Cosmos.catch_fatal_exception do
        unless option_parser and options
          option_parser, options = create_default_options()
          options.width = 550
          options.height = 500
          options.title = "COSMOS Tool Box"
          options.config_file = 'cosmos_tool_box.txt'
          option_parser.separator "Cosmos Tool Box Specific Options:"
          option_parser.on("-c", "--config FILE", "Use the specified configuration file") do |arg|
            options.config_file = arg
          end
        end

        super(option_parser, options)
      end
    end

  end # class DataViewer

end

