/*
 * ubx_msgs.h
 *
 *  Created on: Jan 29, 2025
 *      Author: eracing
 */

#ifndef UBLOX_GPS_INC_UBX_MSGS_H_
#define UBLOX_GPS_INC_UBX_MSGS_H_
#include <stdint.h>
#pragma pack(push, 1)  // Ensure no padding is added to any struct for alignment

typedef struct {
	uint32_t iTow;       // GPS time of week in milliseconds
	uint16_t year;       // Year (UTC)
	uint8_t month;       // Month, range 1..12 (UTC)
	uint8_t day;         // Day of month, range 1..31 (UTC)
	uint8_t hour;        // Hour of day, range 0..23 (UTC)
	uint8_t min;         // Minute of hour, range 0..59 (UTC)
	uint8_t sec;         // Seconds of minute, range 0..60 (UTC)
	uint8_t valid;       // Validity flags (bitfield)
	uint32_t tAcc;       // Time accuracy estimate in nanoseconds
	int32_t nano;        // Fraction of second in nanoseconds, range -1e9 .. 1e9
	uint8_t fixType;     // GNSS fix type
	uint8_t flags;       // Fix status flags (bitfield)
	uint8_t flags2;      // Additional flags (bitfield)
	uint8_t numSV;       // Number of satellites used in solution
	int32_t lon;         // Longitude in 1e-7 degrees
	int32_t lat;         // Latitude in 1e-7 degrees
	int32_t height;      // Height above ellipsoid in millimeters
	int32_t hMSL;        // Height above mean sea level in millimeters
	uint32_t hAcc;       // Horizontal accuracy estimate in millimeters
	uint32_t vAcc;       // Vertical accuracy estimate in millimeters
	int32_t velN;        // North velocity in millimeters per second
	int32_t velE;        // East velocity in millimeters per second
	int32_t velD;        // Down velocity in millimeters per second
	int32_t gSpeed;      // Ground speed in millimeters per second
	int32_t headMot;     // Heading of motion in 1e-5 degrees
	uint32_t sAcc;       // Speed accuracy estimate in millimeters per second
	uint32_t headAcc;    // Heading accuracy estimate in 1e-5 degrees
	uint16_t pDOP;       // Position DOP (scaled by 0.01)
	uint8_t flags3;      // Additional flags (bitfield)
	int32_t headVeh;     // Heading of vehicle in 1e-5 degrees
	int16_t magDec;      // Magnetic declination in 1e-2 degrees
	uint16_t magAcc;     // Magnetic declination accuracy in 1e-2 degrees
} ubx_nav_pvt_t;


typedef struct{
	uint8_t portID;
	uint8_t reserved0;
	uint16_t txReady;
	uint32_t  mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint8_t reserved[2];
}ubx_cfg_prt;



typedef struct {
	uint8_t version;
	uint8_t layers;
	uint8_t reserved0[2];
} ubx_cfg_valset_t;



#pragma pack(pop)  // Restore the packing alignment
#endif /* UBLOX_GPS_INC_UBX_MSGS_H_ */
