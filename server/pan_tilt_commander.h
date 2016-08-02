#ifndef  PAN_TILT_COMMANDER
#define  PAN_TILT_COMMANDER

#include "Log.h"
#include "PacketSubscriber.h"
#include "PanTiltThread.h"
#include "ScannerDefs.h"

class PanTiltCommander : public coral::netapp::PacketSubscriber {
public:

	PanTiltCommander( PanTiltThread& pan_tilt )
		: pan_tilt_( pan_tilt )
	{}

	~PanTiltCommander() {}

	bool put( DestinationID destination_id, const void* data_ptr, ui32 length ) {

		const scanner_flat_defs::message* message_ptr =
			reinterpret_cast<const scanner_flat_defs::message*>(data_ptr);

		switch ( message_ptr->header.type )
      {
         case scanner_flat_defs::MODE_IDLE:
            pan_tilt_.set_mode( PanTiltThread::kIdle );
            break;

         case scanner_flat_defs::MODE_POINT:
         	{
		         const scanner_mode& mode_attrs = message_ptr->data.mode;
	            pan_tilt_.set_stare_point( mode_attrs.phi, mode_attrs.theta );
	         }
            break;

         case scanner_flat_defs::MODE_RASTER:
         	{
         		const scanner_mode& mode_attrs = message_ptr->data.mode;
	            pan_tilt_.set_min_phi(mode_attrs.min_phi);
	            pan_tilt_.set_max_phi(mode_attrs.max_phi);
	            pan_tilt_.set_min_theta(mode_attrs.min_theta);
	            pan_tilt_.set_max_theta(mode_attrs.max_theta);
	            pan_tilt_.set_mode( PanTiltThread::kRaster );
	         }
            break;

         default:
            log::error("Invalid scanner mode requested!\n");
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

   	struct timeval now;
   	gettimeofday(&now,nullptr);

   	response.data.status.timestamp.sec = now.tv_sec;
   	response.data.status.timestamp.usec = now.tv_usec;

   	GenericPacket* packet_ptr = new GenericPacket(
   		scanner_flat_defs::full_message_size( response ) );
   	memcpy(packet_ptr->data_ptr(),&response,scanner_flat_defs::full_message_size( response ));

   	sendTo( Scanner::kCommanderSubscription, packet_ptr );
   }

private:

	PanTiltThread& pan_tilt_;
};

#endif // PAN_TILE_COMMANDER