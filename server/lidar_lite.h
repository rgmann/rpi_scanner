
#ifndef  LIDAR_LITE_H
#define  LIDAR_LITE_H

class I2cInterface;

class LidarLite {
public:

   static const uint8_t kDeviceAddress = 0x62;

   LidarLite( I2cInterface* i2c, uint8_t address = kDeviceAddress );
   ~LidarLite();

   bool eye_safe() { read_mode_status() & 0x80; } ;
   bool good() { read_mode_status() & 0x20; };
   bool busy() { read_mode_status() & 0x01; };

   float get_range( bool corrected = true );

   uint8_t read_mode_status();
   uint8_t read_health_status();

private:

   bool write( uint8_t reg, uint8_t value );

   bool busy_wait();

   bool read( uint8_t reg, uint8_t& value );
   bool read( uint8_t reg, size_t count, uint8_t* value );

private:

   static const uint8_t kWriteAddress = 0xC4;
   static const uint8_t kReadAddress = 0xC5;

   static const uint8_t kRegControl = 0x00;
   static const uint8_t kRegStatus = 0x01;
   static const uint8_t kRegMaxAcqCount = 0x02;
   static const uint8_t kRegCorrRecLenSetting = 0x03;
   static const uint8_t kRegAcqModeCtrl = 0x04;
   static const uint8_t kRegMeasVelocity = 0x09;
   static const uint8_t kRegReceivedSignalStrength = 0x0E;
   static const uint8_t kRegCalcDistanceL = 0x10;
   static const uint8_t kRegCalcDistanceH = 0x0F;
   static const uint8_t kRegDistanceCal = 0x13;
   static const uint8_t kRegPrevDistanceL = 0x14;
   static const uint8_t kRegPrevDistanceH = 0x15;

   I2cInterface* i2c_;
   uint8_t address_;

};

#endif // LIDAR_LITE_H
