#ifndef  SCANNER_POINT_CALLBACK
#define  SCANNER_POINT_CALLBACK

#include "PacketSubscriber.h"
#include "pan_tilt_thread.h"
#include "scanner_point.h"
#include "scanner_flat_defs.h"

class ScannerPointCallback :
public PanTiltThread::MeasurementCallback,
public coral::netapp::PacketSubscriber {
public:

   void operator()( PanTiltThread::ControlMode mode, const PanTiltThread::Point& point ) {
      coral::netapp::GenericPacket* point_packet = new coral::netapp::GenericPacket(
			sizeof(scanner_flat_defs::message_header),
			sizeof(scanner_flat_defs::scanner_point));

		scanner_flat_defs::message_header* message_hdr =
			reinterpret_cast<scanner_flat_defs::message_header*>(point_packet->basePtr());
		memcpy(&message_hdr->marker, scanner_flat_defs::MARKER, sizeof(scanner_flat_defs::MARKER));
		message_hdr->size = sizeof(scanner_flat_defs::scanner_point);
		message_hdr->type = scanner_flat_defs::POINT;

		scanner_flat_defs::scanner_point* point_ptr =
			reinterpret_cast<scanner_flat_defs::scanner_point*>(point_packet->dataPtr());

		struct timeval ts;
		gettimeofday(&ts,NULL);

		point_ptr->timestamp.sec = ts.tv_sec;
		point_ptr->timestamp.usec = ts.tv_usec;
      point_ptr->phi   = point.phi;
      point_ptr->theta = point.theta;
      point_ptr->r     = point.r;

      sendTo( Scanner::kPointSubscription, point_packet );
   };

   bool put( coral::netapp::DestinationID destination_id, const void* data_ptr, ui32 length ) {
      return false;
   }
};

#endif // SCANNER_POINT_CALLBACK
