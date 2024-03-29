#ifndef  PAN_TILT_COMMANDER
#define  PAN_TILT_COMMANDER

#include <stdio.h>
#include <sstream>

#include "Log.h"
#include "GenericPacket.h"
#include "PacketSubscriber.h"
#include "pan_tilt_thread.h"
#include "scanner_flat_defs.h"
#include "scanner_point.h"

class PanTiltCommander : public coral::netapp::PacketSubscriber {
public:

	PanTiltCommander( PanTiltThread& pan_tilt )
		: pan_tilt_( pan_tilt )
	{}

	~PanTiltCommander() {}

	bool put( coral::netapp::DestinationID destination_id, const void* data_ptr, ui32 length ) {

		const scanner_flat_defs::message* message_ptr =
			reinterpret_cast<const scanner_flat_defs::message*>(data_ptr);

		switch ( message_ptr->header.type )
      {
			case scanner_flat_defs::SET_MODE:
				switch ( message_ptr->data.mode.mode )
				{
		         case scanner_flat_defs::MODE_IDLE:
		            pan_tilt_.set_mode( PanTiltThread::kIdle );
		            break;
		
		         case scanner_flat_defs::MODE_POINT:
		         	{
							const scanner_flat_defs::scanner_mode& mode_attrs = message_ptr->data.mode;
							pan_tilt_.set_stare_point( mode_attrs.phi, mode_attrs.theta );
			            pan_tilt_.set_mode( PanTiltThread::kStare );
			         }
		            break;
		
		         case scanner_flat_defs::MODE_RASTER:
		         	{
		         		const scanner_flat_defs::scanner_mode& mode_attrs = message_ptr->data.mode;
			            pan_tilt_.set_mode( PanTiltThread::kRaster );
			         }
		            break;
		
		         default:
		            break;
				}
				break;

			case scanner_flat_defs::SET_LIMITS:
				{
			      const scanner_flat_defs::scanner_limits& mode_attrs = message_ptr->data.limits;
	            pan_tilt_.set_min_phi(mode_attrs.min_phi);
	            pan_tilt_.set_max_phi(mode_attrs.max_phi);
	            pan_tilt_.set_min_theta(mode_attrs.min_theta);
	            pan_tilt_.set_max_theta(mode_attrs.max_theta);
				}
				break;

			case scanner_flat_defs::STATUS_REQUEST:
				break;

			case scanner_flat_defs::SHUTDOWN:
				begin_shutdown( message_ptr->data.shutdown.reboot );
				break;

			default:
				break;
		}

      send_response();

      return true;
   }

   void send_response()
   {
   	scanner_flat_defs::message response;

   	memcpy(&response.header.marker,scanner_flat_defs::MARKER,sizeof(scanner_flat_defs::MARKER));
   	response.header.type = scanner_flat_defs::STATUS_RESPONSE;
   	response.header.size = sizeof(scanner_flat_defs::scanner_status);

   	response.data.status.mode = pan_tilt_.get_mode();
   	response.data.status.min_phi = pan_tilt_.get_min_phi();
   	response.data.status.max_phi = pan_tilt_.get_max_phi();
   	response.data.status.min_theta = pan_tilt_.get_min_theta();
   	response.data.status.max_theta = pan_tilt_.get_max_theta();
   	response.data.status.phi   = pan_tilt_.get_phi();
   	response.data.status.theta = pan_tilt_.get_theta();

		response.data.status.ll_eye_safe = pan_tilt_.get_ll_eye_safe();
		response.data.status.ll_good     = pan_tilt_.get_ll_good();
		response.data.status.ll_status   = pan_tilt_.get_ll_status();

   	struct timeval now;
   	gettimeofday(&now,NULL);

   	response.data.status.timestamp.sec = now.tv_sec;
   	response.data.status.timestamp.usec = now.tv_usec;

   	coral::netapp::GenericPacket* packet_ptr = new coral::netapp::GenericPacket(
			sizeof( scanner_flat_defs::message_header ),
   		response.header.size );
   	memcpy( packet_ptr->basePtr(), &response, scanner_flat_defs::full_message_size( response ));

   	sendTo( Scanner::kCommanderSubscription, packet_ptr );
   }

	void begin_shutdown( bool reboot )
	{

		std::stringstream stream;

		stream << "sudo shutdown ";
		if ( reboot ) stream << "-r";
		else stream << "-h";
		stream << " now";

		FILE* shutdown_cmd = popen( stream.str().c_str(), "r");
		if ( shutdown_cmd )
		{
			pclose( shutdown_cmd );
		}
		else
		{
			coral::log::error("Failed to initiate shutdown\n");
		}
	}

private:

	PanTiltThread& pan_tilt_;
};

#endif // PAN_TILE_COMMANDER
