#ifndef  PAN_TILT_COMMANDER
#define  PAN_TILT_COMMANDER

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

		coral::log::status("DATA: length=%d\n",length);
		const scanner_flat_defs::message* message_ptr =
			reinterpret_cast<const scanner_flat_defs::message*>(data_ptr);

		switch ( message_ptr->header.type )
      {
			case scanner_flat_defs::SET_MODE:
				coral::log::status("SET_MODE\n");
				switch ( message_ptr->data.mode.mode )
				{
		         case scanner_flat_defs::MODE_IDLE:
				coral::log::status("MODE IDLE\n");
		            pan_tilt_.set_mode( PanTiltThread::kIdle );
		            break;
		
		         case scanner_flat_defs::MODE_POINT:
		         	{
							const scanner_flat_defs::scanner_mode& mode_attrs = message_ptr->data.mode;
				coral::log::status("MODE POINT: phi=%0.6f, theta=%0.6f\n",mode_attrs.phi, mode_attrs.theta);
							pan_tilt_.set_stare_point( mode_attrs.phi, mode_attrs.theta );
			            pan_tilt_.set_mode( PanTiltThread::kStare );
			         }
		            break;
		
		         case scanner_flat_defs::MODE_RASTER:
		         	{
				coral::log::status("MODE RASTER\n");
		         		const scanner_flat_defs::scanner_mode& mode_attrs = message_ptr->data.mode;
			            pan_tilt_.set_mode( PanTiltThread::kRaster );
			         }
		            break;
		
		         default:
		            coral::log::error("Invalid scanner mode requested!\n");
		            break;
				}
				break;

			case scanner_flat_defs::SET_LIMITS:
				coral::log::status("SET_LIMITS\n");
				{
		      const scanner_flat_defs::scanner_limits& mode_attrs = message_ptr->data.limits;
            pan_tilt_.set_min_phi(mode_attrs.min_phi);
            pan_tilt_.set_max_phi(mode_attrs.max_phi);
            pan_tilt_.set_min_theta(mode_attrs.min_theta);
            pan_tilt_.set_max_theta(mode_attrs.max_theta);
				}
				break;

			case scanner_flat_defs::STATUS_REQUEST:
				coral::log::status("STATUS_REQUEST\n");
				break;

			default:
				coral::log::status("STATUS_REQUEST\n");
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
/*
   	response.data.status.mode = pan_tilt_.get_mode();
   	response.data.status.min_phi = pan_tilt_.get_min_phi();
   	response.data.status.max_phi = pan_tilt_.get_max_phi();
   	response.data.status.min_theta = pan_tilt_.get_min_theta();
   	response.data.status.max_theta = pan_tilt_.get_max_theta();
   	response.data.status.phi   = pan_tilt_.get_phi();
   	response.data.status.theta = pan_tilt_.get_theta();

   	struct timeval now;
   	gettimeofday(&now,NULL);

   	response.data.status.timestamp.sec = now.tv_sec;
   	response.data.status.timestamp.usec = now.tv_usec;
*/
   	coral::netapp::GenericPacket* packet_ptr = new coral::netapp::GenericPacket(
   		scanner_flat_defs::full_message_size( response ) );
   	memcpy(packet_ptr->dataPtr(),&response,scanner_flat_defs::full_message_size( response ));

coral::log::status("SENDING RESPONSE\n");
   	sendTo( Scanner::kCommanderSubscription, packet_ptr );
coral::log::status("RESPONSE SENT\n");
   }

private:

	PanTiltThread& pan_tilt_;
};

#endif // PAN_TILE_COMMANDER
