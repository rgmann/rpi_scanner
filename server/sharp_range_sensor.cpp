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

#include <Log.h>
#include "ads1115_interface.h"
#include "sharp_range_sensor.h"

using namespace coral;

//-----------------------------------------------------------------------------
SharpRangeSensor::SharpRangeSensor(
   Ads1115Interface* adc, size_t channel,
   float min_voltage, float max_voltage,
   float min_range_cm, float max_range_cm
)
   : adc_( adc )
   , channel_( channel )
   , min_voltage_( min_voltage )
   , max_voltage_( max_voltage )
   , min_range_cm_( min_range_cm )
   , max_range_cm_( max_range_cm )
{
   interp_slope_ = ( ( 1.0 / min_range_cm_ ) - ( 1.0 / max_range_cm_ ) ) /
                   ( max_voltage_ - min_voltage_ );
}

//-----------------------------------------------------------------------------
SharpRangeSensor::~SharpRangeSensor()
{
}

//-----------------------------------------------------------------------------
bool SharpRangeSensor::get_range( float& range_cm, bool& in_range )
{
   bool success = false;

   if ( adc_ )
   {
      static const Ads1115Interface::SingleMuxMode CHANNELS[] = {
         Ads1115Interface::kSingleP0,
         Ads1115Interface::kSingleP1,
         Ads1115Interface::kSingleP2,
         Ads1115Interface::kSingleP3
      };

      // Set the active channel.
      if ( adc_->set_single_mode( CHANNELS[ channel_ ] ) )
      {
         if ( adc_->start_conversion() )
         {
            boost::this_thread::sleep(boost::posix_time::milliseconds(2));
            float one_shot_voltage = 0;

            if ( adc_->read_voltage( one_shot_voltage ) )
            {
               // Interpolate the inverse of the range in centimeters.
               float range_inverse_cm =
                  interp_slope_ * ( one_shot_voltage - min_voltage_ ) + 1 / max_range_cm_;

               range_cm = 1.0 / range_inverse_cm;

               in_range = ( ( range_cm >= min_range_cm_ ) && ( range_cm <= max_range_cm_ ) );

               success = true;
            }
            else
            {
               log::error("SharpRangeSensor::get_range: Failed to read voltage.\n");
            }
         }
         else
         {
            log::error("SharpRangeSensor::get_range: Failed to initiate conversion.\n");
         }
      }
      else
      {
         log::error("SharpRangeSensor::get_range: Failed to select ADC channel.\n");
      }
   }
   else
   {
      log::error("SharpRangeSensor::get_range: ADC not set.\n");
   }

   return success;
}
