
// Date: 2015-11-28

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <list>
#include <set>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "Log.h"
#include "ArgParser.h"

#include "i2c_interface.h"
#include "pwm_controller.h"
#include "ads1115_interface.h"
#include "pan_tilt_controller.h"
#include "sharp_range_sensor.h"
#include "pan_tilt_thread.h"

#include "PacketSubscriber.h"
#include "AsioTcpPacketRouter.h"
#include "AsioTcpServer.h"

#include "RpcServer.h"

#include "ScannerRpc.pb.h"
#include "ScannerServiceSetModeAction.h"
#include "ScannerServiceServerStub.h"

#include "scanner_point.h"

using boost::asio::ip::tcp;
using namespace coral;
using namespace coral::netapp;
using namespace coral::cli;
using namespace coral::rpc;

#define  PWM_FREQ_HZ    60

#define  PAN_CHANNEL    0
#define  TILT_CHANNEL   4


class SetModeAction : public ScannerRpc::ScannerServiceSetModeAction {
public:

   SetModeAction( PanTiltThread& pan_tilt )
      : pan_tilt_( pan_tilt ){}

protected:

   virtual void SetMode(
      const ScannerRpc::Request& request, ScannerRpc::Response& response, coral::rpc::RpcException& e)
   {
      log::status("SET_MODE\n");
      response.set_success(true);
      response.set_error("");

      switch ( request.mode() )
      {
         case ScannerRpc::Request_ScannerMode_IDLE:
            pan_tilt_.set_mode( PanTiltThread::kIdle );
            break;

         case ScannerRpc::Request_ScannerMode_POINT:
            pan_tilt_.set_stare_point( request.set_point().phi(), request.set_point().theta() );
            break;

         case ScannerRpc::Request_ScannerMode_RASTER:
            if ( request.has_raster_options() )
            {
               if ( request.raster_options().has_min_phi() )
               {
                  pan_tilt_.set_min_phi(request.raster_options().min_phi());
               }
               if ( request.raster_options().has_max_phi() )
               {
                  pan_tilt_.set_max_phi(request.raster_options().max_phi());
               }
               if ( request.raster_options().has_min_theta() )
               {
                  pan_tilt_.set_min_theta(request.raster_options().min_theta());
               }
               if ( request.raster_options().has_max_theta() )
               {
                  pan_tilt_.set_max_theta(request.raster_options().max_theta());
               }
               if ( request.raster_options().has_measure_delay_ms() )
               {
                  pan_tilt_.set_hover_time(request.raster_options().measure_delay_ms());
               }
            }
            pan_tilt_.set_mode( PanTiltThread::kRaster );
            break;

         default:
            log::error("Invalid scanner mode requested!\n");
            response.set_error("Invalid scanner mode requested!\n");
            response.set_success(false);
            break;
      }
   }

   PanTiltThread& pan_tilt_;
};

//----------------------------------------------------------------------
class ScannerPointCallback :
public PanTiltThread::MeasurementCallback,
public PacketSubscriber {
public:

   void operator()( const PanTiltThread::Point& point ) {
      GenericPacket* point_packet = new GenericPacket(sizeof(Scanner::ScannerPoint));
      Scanner::ScannerPoint* new_point = (Scanner::ScannerPoint*)(point_packet->basePtr());
      new_point->phi   = point.phi;
      new_point->theta = point.theta;
      new_point->r     = point.r;
      sendTo( Scanner::kPointSubscription, point_packet );
   };

   bool put( DestinationID destination_id, const void* data_ptr, ui32 length ) {
      return false;
   }
};

//----------------------------------------------------------------------
class AsioScannerServerSession :
public AsioTcpPacketRouter  {
public:

   AsioScannerServerSession(
      boost::asio::io_service& io_service,
      coral::rpc::RpcServer& rpc_server,
      ScannerPointCallback& point_callback
   )
      : AsioTcpPacketRouter( io_service )
      , rpc_server_( rpc_server )
      , point_callback_( point_callback )
   {
   }

   ~AsioScannerServerSession()
   {
   }

   void afterConnect() {
      AsioTcpPacketRouter::afterConnect();
      log::status("Connection accepted...\n");
      subscribe( Scanner::kApiSubscription, &rpc_server_ );
      subscribe( Scanner::kPointSubscription, &point_callback_ );
   }

   void beforeClose() {
      unsubscribe( Scanner::kApiSubscription, &rpc_server_ );
      unsubscribe( Scanner::kPointSubscription, &point_callback_ );
      log::status("Connection closed...\n");
   }

private:

   coral::rpc::RpcServer& rpc_server_;
   ScannerPointCallback&  point_callback_;
};

typedef boost::shared_ptr<AsioScannerServerSession> AsioScannerServerSessionPtr;


//----------------------------------------------------------------------
class AsioScannerServer : public AsioTcpServer {
public:

   AsioScannerServer( boost::asio::io_service& io_service, const tcp::endpoint& endpoint, PanTiltThread& pan_tilt )
      : AsioTcpServer( io_service, endpoint )
      , pan_tilt_( pan_tilt )
      , rpc_server_( Scanner::kApiSubscription )
      , set_mode_action_( pan_tilt )
   {
      rpc_server_.registerResource( &scanner_service );
      scanner_service.addAction( &set_mode_action_ );

      pan_tilt_.set_callback( &point_callback_ );
   }

   ~AsioScannerServer()
   {
      std::set<AsioTcpPacketRouterPtr>::iterator iter = clients_.begin();
      for (; iter != clients_.end(); ++iter )
      {
         AsioTcpPacketRouterPtr instance = *iter;
         instance->close();
      }

      pan_tilt_.set_callback( NULL );
   }

protected:

   AsioTcpPacketRouterPtr createRouter( boost::asio::io_service& io_service )
   {
      AsioTcpPacketRouterPtr session( new AsioScannerServerSession( io_service, rpc_server_, point_callback_ ) );
      clients_.insert( session );
      return session;
   }

private:

   PanTiltThread& pan_tilt_;

   coral::rpc::RpcServer   rpc_server_;
   ScannerRpc::ScannerServiceServerStub scanner_service;
   SetModeAction set_mode_action_;

   ScannerPointCallback    point_callback_;

   std::set<AsioTcpPacketRouterPtr> clients_;
};

typedef boost::shared_ptr<AsioScannerServer>   AsioScannerServerPtr;
// typedef std::list<AsioRsyncServerPtr>        AsioRsyncServerList;


int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   ArgParser args;
   args.addArg("name: Port, primary: p, alt: port, type: opt, \
               vtype: int, required, desc: Set port number");

   if ( args.parse( (const char**)argv, argc ) )
   {

      I2cInterface* i2c = I2cInterface::instance( "/dev/i2c-1" );

      if ( i2c )
      {
         PwmController pwm( i2c );
         Ads1115Interface adc( i2c );

         if ( pwm.initialize() )
         {
            bool init_success = true;

            if ( pwm.set_frequency( PWM_FREQ_HZ ) == false )
            {
               log::error("Failed to configure PWM frequency.\n");
               init_success = false;
            }

            if ( adc.set_conversion_mode( Ads1115Interface::kOneShot ) == false )
            {
               log::error("Failed to configure one shot ADC mode.\n");
               init_success = false;
            }

            if ( adc.set_gain_mode( Ads1115Interface::kGain1x ) == false )
            {
               log::error("Failed to configure ADC gain mode.\n");
               init_success = false;
            }

            if ( adc.set_comparator_mode( Ads1115Interface::kDisableComparator ) == false )
            {
               log::error("Failed to disable ADC comparator mode.\n");
               init_success = false;
            }

            if ( adc.set_data_rate( Ads1115Interface::kSPS860 ) == false )
            {
               log::error("Failed to set ADC data rate.\n");
               init_success = false;
            }

            uint16_t adc_config_final = 0;
            if ( adc.read_config( adc_config_final ) == false )
            {
               log::error("Failed to read ADC config.\n");
               init_success = false;
            }
            else
            {
               log::status("ADC config final = 0x%04X\n",adc_config_final);
            }

            if ( init_success )
            // if ( pwm.set_frequency( PWM_FREQ_HZ ) )
            {
               PanTiltController pan_tilt( &pwm, PAN_CHANNEL, TILT_CHANNEL );
               SharpRangeSensor  short_range( &adc, 0, 0.43f, 2.41f, 20.0f, 150.0f );
               SharpRangeSensor  long_range( &adc, 1, 1.45f, 2.46f, 100.0f, 500.0f );

               PanTiltThread pan_tilt_thread( pan_tilt, short_range, long_range );
               pan_tilt_thread.launch();

               // Star listening for connections.
               try
               {
                  int host_port = 0;
                  args.getArgVal( Argument::ArgName, "Port", host_port );

                  boost::asio::io_service io_service;
                  tcp::endpoint endpoint( tcp::v4(), host_port );

                  coral::log::status( "Starting server on port %d\n", host_port );
                  AsioScannerServerPtr server( new AsioScannerServer( io_service, endpoint, pan_tilt_thread ) );

                  server->startAccept();

                  io_service.run();
               }
               catch (std::exception& e)
               {
                  coral::log::error( "Exception: %s\n", e.what() );
               }


               pan_tilt_thread.cancel( true );
            }
            else
            {
               log::error("Failed to set PWM frequency.\n");
            }
         }
         else
         {
            log::error("Failed to initialize PWM controller.\n");
         }
      }
      else
      {
         log::error("Failed to open I2C interface.\n");
      }

      I2cInterface::destroy();
   }
   else
   {
      if ( args.helpRequested() )
      {
         coral::log::raw( args.printHelp().c_str() );
      }
      else
      {
         coral::log::raw( args.printArgErrors(true).c_str() );
      }
   }

   log::flush();

   return 0;
}
