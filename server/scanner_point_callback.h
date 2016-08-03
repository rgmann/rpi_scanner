#ifndef  SCANNER_POINT_CALLBACK
#define  SCANNER_POINT_CALLBACK

#include "PacketSubscriber.h"
#include "pan_tilt_thread.h"
#include "scanner_point.h"

class ScannerPointCallback :
public PanTiltThread::MeasurementCallback,
public coral::netapp::PacketSubscriber {
public:

   void operator()( PanTiltThread::ControlMode mode, const PanTiltThread::Point& point ) {
      coral::netapp::GenericPacket* point_packet = new coral::netapp::GenericPacket(sizeof(Scanner::ScannerPoint));
      Scanner::ScannerPoint* new_point = (Scanner::ScannerPoint*)(point_packet->basePtr());
      new_point->phi   = point.phi;
      new_point->theta = point.theta;
      new_point->r     = point.r;
      sendTo( Scanner::kPointSubscription, point_packet );
   };

   bool put( coral::netapp::DestinationID destination_id, const void* data_ptr, ui32 length ) {
      return false;
   }
};

#endif // SCANNER_POINT_CALLBACK
