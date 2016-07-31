// 
// Copyright (c) 2015, Robert Glissmann
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 

// %% license-end-token %%
// 
// Author: Robert.Glissmann@gmail.com (Robert Glissmann)
// Date: 2015-11-27
// 

#include <arpa/inet.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <Log.h>
#include "i2c_interface.h"
#include "ads1115_interface.h"

#define  ADS1115_TO_SCALE(voltage_range) ( voltage_range / 32767.0 )

#define  ADS1115_CONV_MODE_MASK     ((uint16_t)0x0100)
#define  ADS1115_DATA_RATE_OFFSET   ((uint16_t)5)
#define  ADS1115_DATA_RATE_MASK     ((uint16_t)0x00E0)
#define  ADS1115_GAIN_MODE_OFFSET   ((uint16_t)9)
#define  ADS1115_GAIN_MODE_MASK     ((uint16_t)0x0E00)
#define  ADS1115_DIFF_MODE_OFFSET   ((uint16_t)12)
#define  ADS1115_DIFF_MODE_MASK     ((uint16_t)0x7000)
#define  ADS1115_SINGLE_MODE_OFFSET ADS1115_DIFF_MODE_OFFSET
#define  ADS1115_SINGLE_MODE_MASK   ADS1115_DIFF_MODE_MASK
#define  ADS1115_COMP_MODE_OFFSET   ((uint16_t)0)
#define  ADS1115_COMP_MODE_MASK     ((uint16_t)0x0003)
#define  ADS1115_OS_MASK            ((uint16_t)0x8000)
#define  ADS1115_CONFIG_REG_ADDR    ((uint8_t)0x01)
#define  ADS1115_CONV_REG_ADDR      ((uint8_t)0x00)

using namespace coral;

//-----------------------------------------------------------------------------
Ads1115Interface::Ads1115Interface(  I2cInterface* i2c, uint16_t address )
   : i2c_( i2c )
   , address_( address )
{
}

//-----------------------------------------------------------------------------
Ads1115Interface::~Ads1115Interface()
{
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_conversion_mode( ConversionMode mode )
{
   bool success = false;

   uint16_t config = 0;
   if ( read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      if ( mode == kOneShot )
      {
         config |= ADS1115_CONV_MODE_MASK;
      }
      else
      {
         config &= ~ADS1115_CONV_MODE_MASK;
      }
      // log::status("Ads1115Interface::set_conversion_mode: config = 0x%04X\n",config);

      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_data_rate( DataRate rate )
{
   bool success = false;
   bool rate_valid = true;

   uint16_t rate_config = uint16_t(rate) & 0x0007;
   if ( rate > kSPS860 )
   {
      rate_valid = false;
      log::error("Ads1115Interface::set_data_rate: Invalid data rate specified.\n");
   }

   uint16_t config = 0;
   if ( rate_valid && read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      // Clear the data-rate bits.
      config &= ~ADS1115_DATA_RATE_MASK;

      // Set the data-rate bits;
      config |= ( rate_config << ADS1115_DATA_RATE_OFFSET );

      // log::status("Ads1115Interface::set_data_rate: config = 0x%04X\n",config);

      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_gain_mode( GainMode mode )
{
   bool success = false;
   bool mode_valid = true;

   uint16_t gain_config = 0;
   switch ( mode )
   {
      case kGainTwoThirds: gain_config = 0; break;
      case kGain1x: gain_config = 1; break;
      case kGain2x: gain_config = 2; break;
      case kGain4x: gain_config = 3; break;
      case kGain8x: gain_config = 4; break;
      case kGain16x: gain_config = 5; break;
      default:
         mode_valid = false;
         log::error("Ads1115Interface::set_gain_mode: Invalid gain mode specified.\n");
         break;
   }

   uint16_t config = 0;
   if ( mode_valid && read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      // Clear the gain-mode bits.
      config &= ~ADS1115_GAIN_MODE_MASK;

      // Set the gain-mode bits;
      config |= ( gain_config << ADS1115_GAIN_MODE_OFFSET );

      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_diff_mode( DiffMuxMode mode )
{
   bool success = false;
   bool mode_valid = true;

   uint16_t mux_config = 0;
   switch ( mode )
   {
      case kDiffP0N1: mux_config = 0; break;
      case kDiffP0N3: mux_config = 1; break;
      case kDiffP1N3: mux_config = 2; break;
      case kDiffP2N3: mux_config = 3; break;
      default:
         mux_config = false;
         log::error("Ads1115Interface::set_diff_mode: Invalid mux mode specified.\n");
         break;
   }

   uint16_t config = 0;
   if ( mode_valid && read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      // Clear the mux-mode bits.
      config &= ~ADS1115_DIFF_MODE_MASK;

      // Set the mux-mode bits.
      config |= ( mux_config << ADS1115_DIFF_MODE_OFFSET );

      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_single_mode( SingleMuxMode mode )
{
   bool success = false;
   bool mode_valid = true;

   uint16_t mux_config = 0;
   switch ( mode )
   {
      case kSingleP0: mux_config = 0; break;
      case kSingleP1: mux_config = 1; break;
      case kSingleP2: mux_config = 2; break;
      case kSingleP3: mux_config = 3; break;
      default:
         mux_config = false;
         log::error("Ads1115Interface::set_single_mode: Invalid mux mode specified.\n");
         break;
   }

   uint16_t config = 0;
   if ( mode_valid && read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      // Clear the mux-mode bits.
      config &= ~ADS1115_SINGLE_MODE_MASK;

      // Set the mux-mode bits.
      config |= ( ( 4 | mux_config ) << ADS1115_SINGLE_MODE_OFFSET );
      // log::status("Ads1115Interface::set_single_mode: config = 0x%04X\n",config);
      
      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_comparator_mode( ComparatorMode mode )
{
   bool success = false;
   bool mode_valid = true;

   uint16_t comparator_mode = 0;
   switch ( mode )
   {
      case kAssertAfterOne: comparator_mode = 0; break;
      case kAssertAfterTwo: comparator_mode = 1; break;
      case kAssertAfterFour: comparator_mode = 2; break;
      case kDisableComparator: comparator_mode = 3; break;
      default:
         comparator_mode = false;
         log::error("Ads1115Interface::set_comparator_mode: Invalid comparator mode specified.\n");
         break;
   }

   uint16_t config = 0;
   if ( mode_valid && read_config( config ) )
   {
      config &= ~ADS1115_OS_MASK;
      // Clear the comparator-mode bits.
      config &= ~ADS1115_COMP_MODE_MASK;

      // Set the comparator-mode bits.
      config |= ( comparator_mode << ADS1115_COMP_MODE_OFFSET );
      
      success = set_config( config );
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::start_conversion()
{
   bool success = false;

   uint16_t config = 0;
   if ( read_config( config ) )
   {
      if ( config & ADS1115_OS_MASK )
      {
         config |= ADS1115_OS_MASK;
         success = set_config( config );
      }
      else
      {
         log::warn("Ads1115Interface::start_conversion: Conversion already in progress.\n");
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::set_config( uint16_t config )
{
   bool success = false;

   if ( i2c_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         config = htons(config);
         if ( i2c_->write( ADS1115_CONFIG_REG_ADDR, &config, sizeof( config ) )
               == I2cInterface::kSuccess )
         {
            success = true;
         }
         else
         {
            log::error(
               "Ads1115Interface::set_config: "
               "Failed to set configuration register.\n");
         }
      }
      else
      {
         log::error("Ads1115Interface::set_config: Failed to acquire I2C\n");
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::read_config( uint16_t& config )
{
   bool success = false;

   if ( i2c_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         size_t bytes_rcvd = 0;
         config = 0;
         if ( i2c_->read( ADS1115_CONFIG_REG_ADDR, &config, sizeof( config ), bytes_rcvd )
               == I2cInterface::kSuccess )
         {
            config = ntohs(config);
            success = ( bytes_rcvd == sizeof( config ) );
            // log::status("Ads1115Interface::read_config: config = 0x%04X\n",config);
         }
         else
         {
            log::error(
               "Ads1115Interface::read_config: "
               "Failed to read configuration register.\n");
         }
      }
      else
      {
         log::error("Ads1115Interface::read_config: Failed to acquire I2C\n");
      }
   }
   else
   {
      log::error("Ads1115Interface::read_config: No I2C interface\n");
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::read( int16_t& value )
{
   bool success = false;

   if ( i2c_ )
   {
      if ( i2c_->acquire( address_ ) == I2cInterface::kSuccess )
      {
         size_t bytes_rcvd = 0;

         if ( i2c_->read( ADS1115_CONV_REG_ADDR, &value, sizeof( value ), bytes_rcvd )
               == I2cInterface::kSuccess )
         {
            value = ntohs(value);
            success = ( bytes_rcvd == sizeof( value ) );
         }
         else
         {
            log::error(
               "Ads1115Interface::read: "
               "Failed to read configuration register.\n");
         }
      }
      else
      {
         log::error("Ads1115Interface::read: Failed to acquire I2C.\n");
      }
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Ads1115Interface::read_voltage( float& voltage )
{
   bool success = false;

   uint16_t config = 0;
   if ( read_config( config ) )
   {
      static const float DYNAMIC_RANGE[] = {
         ADS1115_TO_SCALE( 6.144 ),
         ADS1115_TO_SCALE( 4.096 ),
         ADS1115_TO_SCALE( 2.048 ),
         ADS1115_TO_SCALE( 1.024 ),
         ADS1115_TO_SCALE( 0.512 ),
         ADS1115_TO_SCALE( 0.256 )
      };

      uint16_t gain_index = ( ( config & ADS1115_GAIN_MODE_MASK ) >> ADS1115_GAIN_MODE_OFFSET );

      // Clamp the gain index.
      if ( gain_index > kGain16x )
      {
         gain_index = kGain16x;
      }

      int16_t raw_adc = 0;
      if ( read( raw_adc ) )
      {
         voltage = DYNAMIC_RANGE[ gain_index ] * float(raw_adc);
         // log::status("Ads1115Interface::read_voltage: gain=%0.6f, raw_adc=0x%04X, voltage=%0.6f\n",
         //    DYNAMIC_RANGE[ gain_index ],raw_adc,voltage);
         success = true;
      }
   }

   return success;
}
