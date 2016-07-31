#ifndef  SCANNER_POINT_H
#define  SCANNER_POINT_H

#include <cmath>
#include <PacketReceiver.h>

namespace Scanner {

const ::coral::netapp::DestinationID kPointSubscription = 15;
const ::coral::netapp::DestinationID kApiSubscription = 13;

struct ScannerPoint {
   float phi;
   float theta;
   float r;

   ScannerPoint() : phi(0), theta(0), r(0.0) {};

   bool operator== ( const ScannerPoint& other )
   {
      return (( phi == other.phi) && (theta == other.theta) && (r == other.r));
   }
};
typedef ScannerPoint ScannerPointSpherical;

struct ScannerPointCartesian {
   float x;
   float y;
   float z;
   ScannerPointCartesian() : x(0), y(0), z(0) {}
   bool from_spherical(const ScannerPointSpherical& spherical) {
      y = spherical.r * cos(spherical.theta);
      x = ( spherical.r * sin(spherical.theta) * cos(spherical.phi) );
      z = ( spherical.r * sin(spherical.theta) * sin(spherical.phi) );
   }
};

};

#endif // SCANNER_POINT_H