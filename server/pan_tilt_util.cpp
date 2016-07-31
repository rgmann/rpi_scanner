
// Date: 2015-11-28

#include "Log.h"
#include "InteractiveCommandRouter.h"

#include "i2c_interface.h"
#include "pwm_controller.h"
#include "ads1115_interface.h"
#include "pan_tilt_controller.h"
#include "lidar_lite.h"
#include "pan_tilt_thread.h"

using namespace coral;
using namespace coral::cli;

#define  PWM_FREQ_HZ    60

#define  PAN_CHANNEL    0
#define  TILT_CHANNEL   4


class IdleCommand : public InteractiveCommand {
public:
   IdleCommand( PanTiltThread& pan_tilt )
      : InteractiveCommand( "idle", "Return to idle state")
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      pan_tilt_.set_mode( PanTiltThread::kIdle );
   }
private:

   PanTiltThread& pan_tilt_;
};
class RasterCommand : public InteractiveCommand {
public:
   RasterCommand( PanTiltThread& pan_tilt )
      : InteractiveCommand( "raster", "Start raster scan")
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      pan_tilt_.set_mode( PanTiltThread::kRaster );
   }
private:

   PanTiltThread& pan_tilt_;
};
class PointCommand : public InteractiveCommand {
public:
   PointCommand( PanTiltThread& pan_tilt )
      : InteractiveCommand( "point", "Set position" )
      , pan_tilt_( pan_tilt ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      if ( args.size() == 2 )
         pan_tilt_.set_stare_point( atof(args[0].c_str()), atof(args[1].c_str()) );
      else {
         log::error("'point' command expects two arguments\n");
      }
   }
private:

   PanTiltThread& pan_tilt_;
};

class RangeCommand : public InteractiveCommand {
public:
   RangeCommand( LidarLite& lidar )
      : InteractiveCommand( "range", "Read ranging sensor" )
      , lidar_( lidar ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      int range = -1;
      log::status("lidar mode/status = 0x%02X\n",lidar_.read_mode_status());
      log::status("lidar health/status = 0x%02X\n",lidar_.read_health_status());
      if ( ( range = lidar_.get_range()) > 0 )
      {
         log::status("range = %d\n",range);
      }
      else {
         log::error("Failed reading range from sensor\n");
      }
   }
private:

   LidarLite& lidar_;
};

class PointCallback : public PanTiltThread::MeasurementCallback {
public:

   void operator()( PanTiltThread::ControlMode mode, const PanTiltThread::Point& point )
   {
      if ( mode == PanTiltThread::kRaster )
         log::status("phi = %0.4f, theta = %0.4f, r = %0.6f\n",point.phi,point.theta,point.r);
   }

};

int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   InteractiveCommandRouter command_router;

   I2cInterface* i2c = I2cInterface::instance( "/dev/i2c-1" );

   if ( i2c )
   {
      PwmController    pwm( i2c );
      // Ads1115Interface adc( i2c );

      if ( pwm.initialize() )
      {
         bool init_success = true;

         if ( pwm.set_frequency( PWM_FREQ_HZ ) == false )
         {
            log::error("Failed to configure PWM frequency.\n");
            init_success = false;
         }

         // if ( adc.set_conversion_mode( Ads1115Interface::kOneShot ) == false )
         // {
         //    log::error("Failed to configure one shot ADC mode.\n");
         //    init_success = false;
         // }

         // if ( adc.set_gain_mode( Ads1115Interface::kGain1x ) == false )
         // {
         //    log::error("Failed to configure ADC gain mode.\n");
         //    init_success = false;
         // }

         // if ( adc.set_comparator_mode( Ads1115Interface::kDisableComparator ) == false )
         // {
         //    log::error("Failed to disable ADC comparator mode.\n");
         //    init_success = false;
         // }

         // if ( adc.set_data_rate( Ads1115Interface::kSPS860 ) == false )
         // {
         //    log::error("Failed to set ADC data rate.\n");
         //    init_success = false;
         // }

         // uint16_t adc_config_final = 0;
         // if ( adc.read_config( adc_config_final ) == false )
         // {
         //    log::error("Failed to read ADC config.\n");
         //    init_success = false;
         // }
         // else
         // {
         //    log::status("ADC config final = 0x%04X\n",adc_config_final);
         // }

         if ( init_success )
         {
            PanTiltController pan_tilt( &pwm, PAN_CHANNEL, TILT_CHANNEL );
            // SharpRangeSensor  short_range( &adc, 0, 0.43f, 2.41f, 20.0f, 150.0f );
            // SharpRangeSensor  long_range( &adc, 1, 1.45f, 2.46f, 100.0f, 500.0f );
            LidarLite lidar( i2c );

            PanTiltThread pan_tilt_thread( pan_tilt, lidar );
	    PointCallback point_callback;
	    pan_tilt_thread.set_callback( &point_callback );
            pan_tilt_thread.launch();

            IdleCommand idle_command( pan_tilt_thread );
            RasterCommand raster_command( pan_tilt_thread );
            PointCommand point_command( pan_tilt_thread );
            RangeCommand range_command( lidar );

            command_router.add( &idle_command );
            command_router.add( &raster_command );
            command_router.add( &point_command );
            command_router.add( &range_command );

            command_router.run();

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

   log::flush();

   return 0;
}
