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
#include "CircularBuffer.h"


using namespace coral::netapp;

typedef boost::shared_ptr<boost::asio::serial_port> AsioSerialPtr;

class AsioSerialPort :
public coral::netapp::PacketRouter,
public coral::netapp::PacketReceiverHook,
public boost::enable_shared_from_this<AsioSerialPort>  {
public:

	AsioSerialPort( boost::asio::io_service& io_service )
		: PacketRouter( this )
		, io_service_( io_service )
		, marker_found_( false )
		, message_bytes_written_(0)
	{
		read_buffer_.allocate( sizeof(scanner_flat_defs::message) );
	}

	~AsioSerialPort() {}

	bool start( const std::string& port_name, uint32_t baud )
	{
		boost::system::error_code ec;

		if (serial_port_) {
			std::cout << "error : port is already opened..." << std::endl;
			return false;
		}

		serial_port_ = AsioSerialPtr(new boost::asio::serial_port(io_service_));
		serial_port_->open( port_name, ec);
		if (ec) {
			std::cout << "error : serial_port_->open() failed...port_name="
				<< port_name << ", e=" << ec.message().c_str() << std::endl; 
			return false;
		}

		// option settings...
		serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud));
		serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
		serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

		async_read_some_();

		return true;
	}

	void stop()
	{
		boost::mutex::scoped_lock guard(lock_);

		if ( serial_port_ )
		{
			serial_port_->cancel();
			serial_port_->close();
			serial_port_.reset();
		}
	}

	bool call( PacketContainer* container_ptr )
	{
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
	      &AsioSerialPort::do_write,
	      shared_from_this(),
	      packet_ptr
	   ));
	}

	void do_write( NetAppPacket* packet_ptr )
	{
		boost::mutex::scoped_lock guard(lock_);

		if ( serial_port_.get() && serial_port_->is_open() )
		{
		   bool write_in_progress = ( write_packets_.empty() == false );

		   write_packets_.push_back( packet_ptr );

		   if ( !write_in_progress )
		   {
		      serial_port_->async_write_some( 
		         boost::asio::buffer(
		            write_packets_.front()->basePtr(),
		            write_packets_.front()->allocatedSize()
		         ),
		         boost::bind(
		            &AsioSerialPort::handle_write,
		            shared_from_this(),
		            boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred
		         )
		      );
		   }
		} else {
			delete packet_ptr;
		}
	}

	void handle_write( const boost::system::error_code& error, size_t bytes_written )
	{
	   if (!error)
	   {
			message_bytes_written_ += bytes_written;

			if ( message_bytes_written_ == write_packets_.front()->allocatedSize() )
			{
				message_bytes_written_ = 0;

				if (!write_packets_.empty() && ( write_packets_.front() != NULL ) )
				{
					delete write_packets_.front();
					write_packets_.front() = NULL;
				}

				write_packets_.pop_front();
				if ( !write_packets_.empty() )
				{
					serial_port_->async_write_some(
						boost::asio::buffer(
							write_packets_.front()->basePtr(),
							write_packets_.front()->allocatedSize()
						),
						boost::bind(
							&AsioSerialPort::handle_write,
							shared_from_this(),
							boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred
						)
					);
				}
			}
	   }
	   else
	   {
			stop();
	   }
	}

	void async_read_some_()
	{
		if (serial_port_.get() == NULL || !serial_port_->is_open()) return;

		serial_port_->async_read_some( 
			boost::asio::buffer(raw_read_buffer_, sizeof(raw_read_buffer_)),
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

		coral::log::status("received %u bytes\n",bytes_transferred);
		read_buffer_.write( raw_read_buffer_, bytes_transferred );

		while ( read_buffer_.size() >= sizeof(scanner_flat_defs::message_header)) {

			if ( marker_found_ == false ) {
				marker_found_ = search_for_marker();
			}

			if ( marker_found_ ) {
				scanner_flat_defs::message message;
				read_buffer_.peek( &message.header, sizeof(message.header) );

				size_t expected_full_message_size = scanner_flat_defs::full_message_size(message);
				coral::log::status("received message header: size=%u, type=%d, full_size=%u\n",
					message.header.size, message.header.type, expected_full_message_size );

				if ( read_buffer_.size() >= expected_full_message_size ) {
					size_t bytes_read = read_buffer_.read( &message, expected_full_message_size );
					coral::log::status("read full message = %u bytes\n",
						bytes_read );
					read_buffer_.read( &message, expected_full_message_size );
					process_message( message );
					marker_found_ = false;
				}
			}
		}

		async_read_some_();
	}

	bool search_for_marker()
	{
		enum marker_detect_states {
			SEARCHING,
			FOUND_0,
			FOUND_1,
			FOUND_2,
			FOUND_MARKER
		} marker_detect_state = SEARCHING;

		uint8_t marker_buffer[sizeof(scanner_flat_defs::MARKER)];

		while ( marker_detect_state != FOUND_MARKER ) {

			if ( read_buffer_.peek( &marker_buffer, sizeof( marker_buffer ) ) != sizeof( marker_buffer ) )
			{
				break;
			}

			marker_detect_state = SEARCHING;

         for ( size_t pos = 0; pos < sizeof( marker_buffer ); ++pos )
			{
				uint8_t byte = marker_buffer[ pos ];

	         coral::log::status("rx: %02X\n",byte);

				switch ( marker_detect_state ) {
					case SEARCHING:
						if ( byte == scanner_flat_defs::MARKER[0] ) {
							marker_detect_state = FOUND_0;
							coral::log::status("FOUND_0\n");
						} else {
							marker_detect_state = SEARCHING;
							coral::log::status("SEARCHING\n");
						}
						break;
					case FOUND_0:
						if ( byte == scanner_flat_defs::MARKER[1] ) {
							marker_detect_state = FOUND_1;
							coral::log::status("FOUND_1\n");
						} else {
							marker_detect_state = SEARCHING;
							coral::log::status("SEARCHING\n");
						}
						break;
					case FOUND_1:
						if ( byte == scanner_flat_defs::MARKER[2] ) {
							marker_detect_state = FOUND_2;
							coral::log::status("FOUND_2\n");
						} else {
							marker_detect_state = SEARCHING;
							coral::log::status("SEARCHING\n");
						}
						break;
					case FOUND_2:
						if ( byte == scanner_flat_defs::MARKER[3] ) {
							marker_detect_state = FOUND_MARKER;
							coral::log::status("FOUND_MARKER\n");
						} else {
							marker_detect_state = SEARCHING;
							coral::log::status("SEARCHING\n");
						}
						break;
				}
			}

			if ( marker_detect_state != FOUND_MARKER ) {
				uint8_t byte[] = {0};
				read_buffer_.read( byte, sizeof(byte) );
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

	AsioSerialPtr 					serial_port_;
	boost::mutex 					lock_;

	char 								raw_read_buffer_[ sizeof(scanner_flat_defs::message) ];
	coral::CircularBuffer					read_buffer_;
	std::deque<NetAppPacket*>   write_packets_;

   bool marker_found_;
	size_t message_bytes_written_;
};

typedef boost::shared_ptr<AsioSerialPort> AsioSerialPortPtr;


#endif // ASIO_SERIAL_PORT
