#include "i2c_interface.h"
#include "lidar_lite.h"

//-----------------------------------------------------------------------------
LidarLite::LidarLite( I2cInterface* i2c )
   : i2c_( i2c )
{
}

//-----------------------------------------------------------------------------
LidarLite::LidarLite() {}

//-----------------------------------------------------------------------------
uint8_t LidarLite::read_mode_status()
{
   uint8_t mode_status = 0;
   read( kRegStatus, mode_status );
   return mode_status;
}

//-----------------------------------------------------------------------------
int LidarLite::get_range( bool corrected )
{
   int distance = -1;

   if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
   {
      // Trigger collection.
      if ( write( kRegControl, corrected ? 0x04 : 0x03 ) )
      {
         uint8_t data[2];
         if ( read( kRegCalcDistanceL & 0x80, sizeof(data), data ) )
         {
            distance = ( data[1] << 8 ) | data[0];
         }
      }
   }

   return distance;
}

//-----------------------------------------------------------------------------
void LidarLite::write( uint8_t reg, uint8_t value )
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
void LidarLite::read( uint8_t reg, uint8_t& value )
{
   I2cInterface::i2c_error status = I2cInterface::kSuccess;
   bool error = false;

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
void LidarLite::read( uint8_t reg, size_t count, uint8_t* value )
{
   I2cInterface::i2c_error status = I2cInterface::kSuccess;
   bool error = false;

   error = !busy_wait();

   if ( !error )
   {
      size_t received_bytes = 0;
      status = i2c_->read( 0x01, &value, count, received_bytes );
      error = ( status != I2cInterface::kSuccess ) || ( received_bytes < count );
   }

   return !error;
}
