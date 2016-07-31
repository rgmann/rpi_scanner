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


#ifndef  ADS_1115_INTERFACE_H
#define  ADS_1115_INTERFACE_H

#include <stdint.h>
#include <cstddef>

class I2cInterface;

class Ads1115Interface {
public:

   static const uint16_t kDefaultAddress = 0x48;
   Ads1115Interface( I2cInterface* i2c, uint16_t address = kDefaultAddress );
   ~Ads1115Interface();

   enum ConversionMode {
      kOneShot    = 0,
      kContinuous
   };
   bool set_conversion_mode( ConversionMode mode );

   enum DataRate {
      kSPS8 = 0,
      kSPS16,
      kSPS32,
      kSPS64,
      kSPS128,
      kSPS250,
      kSPS475,
      kSPS860
   };
   bool set_data_rate( DataRate rate );

   enum GainMode {
      kGainTwoThirds = 0,
      kGain1x,
      kGain2x,
      kGain4x,
      kGain8x,
      kGain16x
   };
   bool set_gain_mode( GainMode mode );

   enum DiffMuxMode {
      kDiffP0N1 = 0,
      kDiffP0N3,
      kDiffP1N3,
      kDiffP2N3
   };
   bool set_diff_mode( DiffMuxMode mode );

   enum SingleMuxMode {
      kSingleP0 = 0,
      kSingleP1,
      kSingleP2,
      kSingleP3
   };
   bool set_single_mode( SingleMuxMode mode );

   enum ComparatorMode {
      kAssertAfterOne = 0,
      kAssertAfterTwo,
      kAssertAfterFour,
      kDisableComparator
   };
   bool set_comparator_mode( ComparatorMode mode );

   bool start_conversion();

   bool set_config( uint16_t config );
   bool read_config( uint16_t& config );

   bool read( int16_t& value );
   bool read_voltage( float& voltage );

private:

   I2cInterface* i2c_;

   uint16_t address_;
};


#endif // ADS_1115_INTERFACE_H
