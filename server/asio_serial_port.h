#ifndef  ASIO_SERIAL_PORT
#define  ASIO_SERIAL_PORT

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>

#include "PacketRouter.h"
#include "PacketReceiverHook.h"
#include "NetAppPacket.h"
#include "scanner_point_callback.h"
#include "pan_tilt_commander.h"


using namespace coral::netapp;

typedef boost::shared_ptr<boost::asio::serial_port> AsioSerialPtr;

class AsioSerialPort :
public coral::netapp::PacketRouter,
public coral::netapp::PacketReceiverHook,
public boost::enable_shared_from_this<AsioSerialPort>  {
public:

	AsioSerialPort( boost::asio::io_service& io_service, ScannerPointCallback& point_callback, PanTiltCommander& pan_tilt_commander )
		: io_service_( io_service )
		, point_callback_( point_callback )
		, pan_tilt_commander_( pan_tilt_commander )
	{
	}

	~AsioSerialPort() {}

	bool start( const std::string& port_name, uint32_t baud )
	{
		boost::system::error_code ec;

		if (serial_port_) {
			std::cout << "error : port is already opened..." << std::endl;
			return false;
		}

		serial_port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
		serial_port_->open(com_port_name, ec);
		if (ec) {
			std::cout << "error : serial_port_->open() failed...com_port_name="
				<< com_port_name << ", e=" << ec.message().c_str() << std::endl; 
			return false;
		}

		// option settings...
		serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud));
		serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
		serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

		subscribe( Scanner::kCommanderSubscription, &pan_tilt_commander_ );
      subscribe( Scanner::kPointSubscription, &point_callback_ );

		async_read_some_();

		return true;
	}

	void stop()
	{
		boost::mutex::scoped_lock look(lock_);

		if ( serial_port_ )
		{
			subscribe( Scanner::kCommanderSubscription, &pan_tilt_commander_ );
      	subscribe( Scanner::kPointSubscription, &point_callback_ );

			serial_port_->cancel();
			serial_port_->close();
			serial_port_.reset();
		}
	}

	bool call( PacketContainer* container_ptr )
	{
	   boost::mutex::scoped_lock guard( lock_ );

	   bool push_success = false;

	   if ( container_ptr )
	   {
	      write_packet( container_ptr );

	      delete container_ptr;
	      push_success = true;
	   }

	   return push_success;
	}

	void write_packet( const PacketContainer* container_ptr )
	{
	   NetAppPacket* packet_ptr = new NetAppPacket(
	      container_ptr->destination_id_,
	      container_ptr->packet_ptr_->allocatedSize()
	   );

	   memcpy( packet_ptr->dataPtr(),
	         container_ptr->packet_ptr_->basePtr(),
	         container_ptr->packet_ptr_->allocatedSize() );

	   io_service_.post( boost::bind(
	      &AsioSerialPort::doWrite,
	      shared_from_this(),
	      packet_ptr
	   ));
	}

	void do_write( NetAppPacket* packet_ptr )
	{
	   bool write_in_progress = ( write_packets_.empty() == false );

	   write_packets_.push_back( packet_ptr );

	   if ( !write_in_progress )
	   {
	      boost::asio::async_write( serial_port_,
	         boost::asio::buffer(
	            write_packets_.front()->basePtr(),
	            write_packets_.front()->allocatedSize()
	         ),
	         boost::bind(
	            &AsioSerialPort::handle_write,
	            shared_from_this(),
	            boost::asio::placeholders::error
	         )
	      );
	   }
	}

	void handle_write( const boost::system::error_code& error )
	{
	   if (!error)
	   {
	      if (!write_packets_.empty() && ( write_packets_.front() != NULL ) )
	      {
	         delete write_packets_.front();
	         write_packets_.front() = NULL;
	      }

	      write_packets_.pop_front();
	      if ( !write_packets_.empty() )
	      {
	         boost::asio::async_write( socket_,
	            boost::asio::buffer(
	               write_packets_.front()->basePtr(),
	               write_packets_.front()->allocatedSize()
	            ),
	            boost::bind(
	               &AsioSerialPort::handleWrite,
	               shared_from_this(),
	               boost::asio::placeholders::error
	            )
	         );
	      }
	   }
	   else
	   {
	      doClose();
	   }
	}

	void async_read_some_()
	{
		if (serial_port_.get() == NULL || !serial_port_->is_open()) return;

		serial_port_->async_read_some( 
			boost::asio::buffer(raw_read_buffer_, MAX_MESSAGE_SIZE),
			boost::bind(
				&AsioSerialPort::handle_read_some,
				this, boost::asio::placeholders::error, 
				boost::asio::placeholders::bytes_transferred));
	}

	void handle_read_some(const boost::system::error_code& ec, size_t bytes_transferred)
	{
		boost::mutex::scoped_lock look(lock_);

		if (serial_port_.get() == NULL || !serial_port_->is_open()) return;
		if (ec) {
			async_read_some_();
			return;
		}

		read_buffer_.write( raw_read_buffer_, bytes_transferred );

		if ( marker_found_ ) {
			scanner_flat_defs::message header;
			read_buffer_.peek( &message.header, sizeof(message.header) );

			if ( read_buffer_.size() == message.header.size ) {
				read_buffer_.read( &message, sizeof(message.header) + message.header.size );
				process_message( message );
			}
		} else {
			marker_found_ = search_for_marker();
		}

		async_read_some_();
	}

	bool search_for_marker()
	{
		enum marker_detect_states {
			SEARCHING,
			FOUND_0,
			FOUND_1,
			FOUND_3,
			FOUND_MARKER
		} marker_detect_state = SEARCHING;

		size_t flush_count = 0;

		uint8_t marker_buffer[sizeof(scanner_flat_defs::MARKER)];
		size_t  marker_read_pos = 0;

		while ( true ) {

			uint8_t byte[] = {0};

			if ( read_buffer_.peek( marker_buffer[marker_read_pos++], 1 ) != 1 )
			{
				break;
			}

			flush_count++;

			switch ( marker_detect_state ) {
				case SEARCHING:
					if ( marker_buffer[0] == MARKER[0] )
						marker_detect_state = FOUND_0;
					else
						marker_detect_state = SEARCHING;
					break;
				case FOUND_0:
					if ( marker_buffer[1] == MARKER[1] )
						marker_detect_state = FOUND_1;
					else
						marker_detect_state = SEARCHING;
					break;
				case FOUND_1:
					if ( marker_buffer[2] == MARKER[2] )
						marker_detect_state = FOUND_2;
					else
						marker_detect_state = SEARCHING;
					break;
				case FOUND_2:
					if ( marker_buffer[3] == MARKER[3] )
						marker_detect_state = FOUND_MARKER;
					else
						marker_detect_state = SEARCHING;
					break;
			}

			// Flush non-marker bytes from the buffer until the marker is found.
			if ( marker_detect_state == SEARCHING ) {
				marker_read_pos = 0;
				if ( flush_count > 0 ) {
					for ( size_t flush_index = 0; flush_index < flush_count; ++flush_index ) {
						read_buffer_.read( byte, sizeof(byte) );
					}
				}
			}
		}

		return ( marker_detect_state == FOUND_MARKER );
	}

	void process_message( const scanner_flat_defs::message& new_message ) {
		publish(
         Scanner::kCommanderSubscription,
         &new_message,
         scanner_flat_defs::full_message_size(new_message) );
	}

private:

	boost::asio::io_service& 	io_service_;
	PanTiltCommander& 			pan_tilt_commander_;
	ScannerPointCallback& 		point_callback_;

	AsioSerialPtr 					serial_port_;
	boost::mutex 					lock_;

	char 								read_message_buffer_[ MAX_MESSAGE_SIZE ];
	char 								read_buffer_[ MAX_MESSAGE_SIZE ];
	std::deque<NetAppPacket*>   write_packets_;
};

typedef boost::shared_ptr<AsioSerialPort> AsioSerialPortPtr;


#endif // ASIO_SERIAL_PORT
