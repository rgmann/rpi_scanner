
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
#include "lidar_lite.h"
#include "pan_tilt_thread.h"

#include "scanner_flat_defs.h"
#include "pan_tilt_commander.h"
#include "asio_serial_port.h"

#include "scanner_point.h"
#include "scanner_point_callback.h"

using boost::asio::ip::tcp;
using namespace coral;
using namespace coral::netapp;
using namespace coral::cli;

#define  PWM_FREQ_HZ    60

#define  PAN_CHANNEL    0
#define  TILT_CHANNEL   4

/*
int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   ArgParser args;
   args.addArg("name: Port, primary: p, alt: port, type: opt, \
               vtype: string, required, desc: serial port name");

   if ( args.parse( (const char**)argv, argc ) )
   {
		try
		{
		   std::string port_name;
		   args.getArgVal( Argument::ArgName, "Port", port_name );
		
			boost::asio::io_service io_service;
		   coral::log::status( "Starting server on port %s\n", port_name.c_str() );
		   AsioSerialPortPtr port_router( new AsioSerialPort( io_service ) );
		
		   port_router->start( port_name, 115200 );
		
		   io_service.run();
		
		   port_router->stop();
		}
		catch (std::exception& e)
		{
		   coral::log::error( "Exception: %s\n", e.what() );
		}
	}

	return 0;
}
*/


int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   ArgParser args;
   args.addArg("name: Port, primary: p, alt: port, type: opt, \
               vtype: string, required, desc: serial port name");

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

            if ( init_success )
            {
               PanTiltController pan_tilt( &pwm, PAN_CHANNEL, TILT_CHANNEL );
               LidarLite lidar( i2c );

               PanTiltThread pan_tilt_thread( pan_tilt, lidar );
               ScannerPointCallback    point_callback;
               PanTiltCommander        pan_tilt_commander( pan_tilt_thread );
               pan_tilt_thread.set_callback( &point_callback );

               pan_tilt_thread.launch();

               // Star listening for connections.
               try
               {
                  std::string port_name;
                  args.getArgVal( Argument::ArgName, "Port", port_name );

						boost::asio::io_service io_service;
                  coral::log::status( "Starting server on port %s\n", port_name.c_str() );
                  AsioSerialPortPtr port_router( new AsioSerialPort( io_service ) );

						port_router->subscribe( Scanner::kCommanderSubscription, &pan_tilt_commander );
						port_router->subscribe( Scanner::kPointSubscription, &point_callback );

                  port_router->start( port_name, 115200 );

                  io_service.run();

                  port_router->stop();


						port_router->unsubscribe( Scanner::kCommanderSubscription, &pan_tilt_commander );
			         port_router->unsubscribe( Scanner::kPointSubscription, &point_callback );

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

