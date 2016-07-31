
// Date: 2015-11-28

#include "Log.h"
#include "InteractiveCommandRouter.h"

#include "i2c_interface.h"
#include "pwm_controller.h"
#include "ads1115_interface.h"
#include "pan_tilt_controller.h"
#include "sharp_range_sensor.h"
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
class ShortRangeCommand : public InteractiveCommand {
public:
   ShortRangeCommand( SharpRangeSensor& range )
      : InteractiveCommand( "short_range", "Read short range sensor" )
      , range_( range ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      float range_cm = 0;
      bool in_range = false;
      if ( range_.get_range( range_cm, in_range ) )
      {
         log::status("%0.6f cm\n",range_cm);
      }
      else {
         log::error("Failed reading range from sensor\n");
      }
   }
private:

   SharpRangeSensor& range_;
};
class LongRangeCommand : public InteractiveCommand {
public:
   LongRangeCommand( SharpRangeSensor& range )
      : InteractiveCommand( "long_range", "Read short range sensor" )
      , range_( range ) {};
   void process(const coral::cli::ArgumentList& args)
   {
      float range_cm = 0;
      bool in_range = false;
      if ( range_.get_range( range_cm, in_range ) )
      {
         log::status("%0.6f cm\n",range_cm);
      }
      else {
         log::error("Failed reading range from sensor\n");
      }
   }
private:

   SharpRangeSensor& range_;
};


int main( int argc, char** argv )
{
   coral::log::level( coral::log::Verbose );

   InteractiveCommandRouter command_router;

   I2cInterface* i2c = I2cInterface::instance( "/dev/i2c-1" );

   if ( i2c )
   {
      PwmController    pwm( i2c );
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
         {
            PanTiltController pan_tilt( &pwm, PAN_CHANNEL, TILT_CHANNEL );
            SharpRangeSensor  short_range( &adc, 0, 0.43f, 2.41f, 20.0f, 150.0f );
            SharpRangeSensor  long_range( &adc, 1, 1.45f, 2.46f, 100.0f, 500.0f );

            PanTiltThread pan_tilt_thread( pan_tilt, short_range, long_range );
            pan_tilt_thread.launch();

            IdleCommand idle_command( pan_tilt_thread );
            RasterCommand raster_command( pan_tilt_thread );
            PointCommand point_command( pan_tilt_thread );
            ShortRangeCommand short_range_command( short_range );
            LongRangeCommand long_range_command( long_range );

            command_router.add( &idle_command );
            command_router.add( &raster_command );
            command_router.add( &point_command );
            command_router.add( &short_range_command );
            command_router.add( &long_range_command );

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
