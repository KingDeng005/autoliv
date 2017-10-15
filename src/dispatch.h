#ifndef _DISPATCH_H
#define _DISPATCH_H
#include <stdint.h>

namespace octopus
{

typedef struct
{
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t type : 4;
  uint8_t : 4;
  uint8_t : 8;
} MsgTargetGeneral

typedef struct
{
  uint8_t range_msb;
  uint8_t range_lsb : 4;
  uint8_t velocity_msb : 4;
  uint8_t velocity_lsb : 6;
  uint8_t bearing_msb : 2;
  uint8_t bearing_lsb : 8;
  uint8_t quality : 4;
  uint8_t track_id : 4;
  uint8_t msg_counter : 4;
  uint8_t sensor_nr : 4;
  uint8_t target_format_type : 4;
  uint8_t bearing_observed_msb : 4;
  uint8_t bearing_observed_lsb : 2;
  uint8_t range_observed : 6;
} MsgTargetPolarShort;

typedef struct
{
  uint8_t range_msb;
  uint8_t range_lsb : 4;
  uint8_t dopper_velocity_msb : 4;
  uint8_t dopper_velocity_lsb : 6;
  uint8_t bearing_msb : 2;
  uint8_t bearing_lsb : 8;
  uint8_t amplitude : 8;
  uint8_t msg_counter : 4;
  uint8_t msg_counter : 4;
  uint8_t target_format_type : 4;
  uint8_t usage : 4;
  uint8_t doppler_alias : 8;
} MsgRawPolarShort;

typedef struct
{
  // TODO
} MsgTargetCartesian;

typedef struct
{
  // TODO
} MsgTargetCartesianMid;

typedef struct
{
  // TODO
} MsgTargetCartesianMul;

typedef struct
{
  // TODO
} MsgFreespaceSegments;

typedef struct
{
  // TODO
} MsgRawPolarLong;

typedef struct
{
  // TODO
} MsgTargetPolarLong;

enum
{
  ID_TARGET_POLAR_SHORT     = 0x1,
  ID_RAW_POLAR_SHORT        = 0x2,
  ID_TARGET_CARTESIAN       = 0x3,
  ID_TARGET_CARTESIAN_MID   = 0x4,
  ID_TARGET_CARTESIAN_MUL   = 0x5,
  ID_FREESPACE_SEGMENTS     = 0x6,
  ID_RAW_POLAR_LONG         = 0x7,
  ID_TARGET_POLAR_LONG      = 0x8
};

}
#endif 
