#ifndef UBX_NAVI
#define UBX_NAVI

// #include <Arduino.h> // For HardwareSerial and sprintf
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
//#include "cmsis_os.h" // for mutex

#define UBX_MAX_PAYLOAD   1000
#define UBX_RX_BUF_SIZE   2000
#define UBX_MAX_numSvs    64

#define X1 uint8_t
#define X2 uint16_t
#define X4 uint32_t
#define U1 uint8_t
#define U2 uint16_t
#define U4 uint32_t
#define I1 int8_t
#define I2 int16_t
#define I4 int32_t

// Navigation Position Velocity Solution
// The UBX-NAV-PVT message is 92+8=100 bytes, while the serial receive buffer is only 64 bytes. 
struct __attribute__((packed, aligned(4))) ubx_nav_pvt_t {
	//U1 class;
	//U1 id;
	//U2 length;
	U4 iTOW;           // GPS time of Week
	U2 year;           // UTC
	U1 month,day,hour,min,sec; // UTC 1..12, 1..31, 0..23, 0..59, 0..60 (so that 0 <= sec+nano < 60?  No, 60 is for leap second)
	X1 valid;          // Validity flags
	U4 tAcc;           // Time accuracy estimate [ns]
	I4 nano;           // Fraction of second -1e9...1e9 [ns] (UTC)
	U1 fixType;        // GNSSfix Type 0:no fix,  1:dead reckoning only,  2:2D-fix,  3:3D-fix,  4:GNSS+dead reck,  5:time only
	X1 flags;          // Fix status flags
	X1 flags2;         // Additional flags
	U1 numSV;          // Number of satellites used in Nav Solution
	I4 lon, lat;       // Longitude and Latitude [1e-7 deg]
	I4 height, hMSL;   // Heigth above ellipsoid and mean sea level [mm]
	U4 hAcc, vAcc;     // Horizontal and vertical accuracy estimate
	I4 velN, velE, velD, gSpeed; // Velocity [mm/s] north, east, down, ground speed (2-D)
	I4 headMot;        // Heading of motion (2-D) [1e-5 deg]
	U4 sAcc;           // Speed accuracy estimate [mm/s]
	U4 headAcc;        // Heading accuracy estimate [1e-5 deg]
	U2 pDOP;           // Position DOP
	U1 reserved1[6];
	I4 headVeh;        // Heading og vehicle (2-D) [1e-5 deg]
	I2 magDec;         // Magnetic declination [1e-2 deg]
	U2 magAcc;         // Magnetic declination accuracy [1e-2 deg]
	//U1 ck_a, ck_b;
};

struct __attribute__((packed, aligned(4))) ubx_nav_sat_grp_t {
	  U1 gnssId;         // GNSS identifier
      U1 svId;           // Satellite identifier
      U1 cno; 		     // [dBHz] Carrier to noise ratio (signal strength)
      I1 elev;           // [deg] Elevation (range: +/-90), unknown if out of range
      I2 azim;			 // [deg] Azimuth (range 0-360), unknown if elevation is out of range
      I2 prRes; 		 // [0.1m] Pseudorange residual
      X4 flags; 		 // Bitmask according to UBX protocol
};

struct __attribute__((packed, aligned(4))) ubx_nav_sat_t {
      U4 iTOW;           // GPS time of Week
      U1 version;        // Message version (0x01 for this version)
      U1 numSvs; 		 // Number of satellites
      U1 reserved0[2];   //
      struct ubx_nav_sat_grp_t svs[UBX_MAX_numSvs];
};

struct __attribute__((packed, aligned(4))) ubx_msg_t {
    U2 state;
    U1 synch1, synch2;
    U1 class;
    U1 id;
    U2 length;
    U1 payload[UBX_MAX_PAYLOAD];
    U1 ck_a, ck_b;  // checksum will be after _actual_ payload not UBX_MAX_PAYLOAD
    U1* c;  // ToDo: move this into ubx_handle_t !
};

struct __attribute__((packed, aligned(4)))ubx_handle_t{
    struct ubx_nav_pvt_t nav_pvt;
    U4 nav_pvt_flag;
    struct ubx_nav_sat_t nav_sat;
    U4 nav_sat_flag;
    struct ubx_msg_t msg;  // latest message to/from GNS module (ToDo: split into rx_msg and tx_msg ?)
    U1 rx_buf[UBX_RX_BUF_SIZE];  // ring buffer for DMA
    size_t rx_pos;  // position (index in ring buffer) of last parsed data
    //osMessageQueueId_t  rx_dma_queue_id;    // ToDo: consider using osMessages
    DMA_TypeDef * rx_dma_master;
    size_t rx_dma_channel;
};

void ubx_init( struct ubx_handle_t * );
void ubx_receive( struct ubx_handle_t * );
void ubx_receive2( struct ubx_handle_t * );
void gngga( const struct ubx_nav_pvt_t*, char [] );
void gnrmc( const struct ubx_nav_pvt_t*, char [] );
void gnzda( const struct ubx_nav_pvt_t*, char [] );
void gnvtg( const struct ubx_nav_pvt_t*, char [] );

#endif
