#include "Log.h"
#include "i2c_interface.h"
#include "lidar_lite.h"

using namespace coral;

//-----------------------------------------------------------------------------
LidarLite::LidarLite( I2cInterface* i2c, uint8_t address )
   : i2c_( i2c )
   , address_( address )
{
   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      write( 0, 0 );
   }
}

//-----------------------------------------------------------------------------
LidarLite::~LidarLite() {}

//-----------------------------------------------------------------------------
uint8_t LidarLite::read_mode_status()
{
   uint8_t mode_status = 0;
   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      read( kRegStatus, mode_status );
   }
   else
   {
      log::error("lidar_lite: failed to acquire interface with address=0x%02X\n",
	 address_);
   }

  return mode_status;
}

//-----------------------------------------------------------------------------
uint8_t LidarLite::read_health_status()
{
   uint8_t health_status = 0;
   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      read( 0x48, health_status );
   }
   else
   {
      log::error("lidar_lite: failed to acquire interface with address=0x%02X\n",
	 address_);
   }

  return health_status;
}

//-----------------------------------------------------------------------------
float LidarLite::get_range( bool corrected )
{
   float distance = -1;

   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      // Trigger collection.
      if ( write( kRegControl, corrected ? 0x04 : 0x03 ) )
      {
         uint8_t data[2] = {0,0};
         if ( read( kRegCalcDistanceL, data[1] ) )
         {
            read( kRegCalcDistanceH, data[0] );
	    uint32_t raw_dist = ( data[0] << 8 ) | data[1];
            distance = (float)raw_dist / (float)0x0000FFFF;
         }
      }
      else
      {
	 log::error("lidar_lite: failed write to register 0x%02X\n",kRegControl);
      }
   }
   else
   {
      log::error("lidar_lite: failed to acquire interface with address=0x%02X\n",
	 address_);
   }

   return distance;
}

//-----------------------------------------------------------------------------
bool LidarLite::write( uint8_t reg, uint8_t value )
{
   return ( i2c_->write( reg, &value, sizeof( value ) ) == I2cInterface::kSuccess );
}

//-----------------------------------------------------------------------------
bool LidarLite::busy_wait()
{
   I2cInterface::i2c_error status = I2cInterface::kSuccess;
   bool error = true;

   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      bool busy = true;
      do {
         uint8_t status_reg = 0;
         size_t received_bytes = 0;
         status = i2c_->read( kRegStatus, &status_reg, sizeof( status_reg ), received_bytes );

         error = (status != I2cInterface::kSuccess) || (received_bytes < sizeof( status_reg ));
         busy = ((status_reg & 0x01) != 0);

      } while( busy && !error );
   }

   return !error;
}

//-----------------------------------------------------------------------------
bool LidarLite::read( uint8_t reg, uint8_t& value )
{
   I2cInterface::i2c_error status = I2cInterface::kSuccess;
   bool error = false;

   value = 0;

   error = !busy_wait();

   if ( !error )
   {
      size_t received_bytes = 0;
      status = i2c_->read( reg, &value, sizeof( value ), received_bytes );
      error = (status != I2cInterface::kSuccess) || (received_bytes < sizeof( value ));
   }

   return !error;
}

//-----------------------------------------------------------------------------
bool LidarLite::read( uint8_t reg, size_t count, uint8_t* value )
{
   I2cInterface::i2c_error status = I2cInterface::kSuccess;
   bool error = false;

   error = !busy_wait();

   if ( !error )
   {
      size_t received_bytes = 0;
      status = i2c_->read( reg, &value, count, received_bytes );
log::status("reading bytes - received_bytes = %d\n",received_bytes);
      error = ( status != I2cInterface::kSuccess ) || ( received_bytes < count );
   }

   return !error;
}

   
