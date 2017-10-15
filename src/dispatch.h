#ifndef _DISPATCH_H
#define _DISPATCH_H
#include <stdint.h>

namespace octopus
{
typedef struct
{
  uint8_t range_msb;
  uint8_t range_lsb : 4;
  uint8_t velocity_msb : 4;
  uint8_t velocity_lsb : 6;
  uint8_t bearing_filtered_msb : 2;
  uint8_t bearing_filtered_lsb : 8;
  uint8_t quality : 4;
  uint8_t track_id : 4;
  uint8_t msg_counter : 4;
  uint8_t sensor_nr : 4;
  uint8_t target_format_type : 4;
  uint8_t bearing_observed_msb : 4;
  uint8_t bearing_observed_msb : 2;
  uint8_t range_observed : 6;
} MsgTargetPolarShort;

typedef struct
{
  uint8_t range_msb;
  uint8_t range_lsb : 4;
  uint8_t dopper_velocity_msb : 4;
  uint8_t dopper_velocity_msb : 6;
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
  
} MsgTargetCartesian;

typedef struct
{
  
} MsgTargetCartesianMid;

typedef struct
{
  
} MsgTargetCartesianMul;

typedef struct
{
  
} MsgFreespaceSegments;

typedef struct
{
  
} MsgRawPolarLong;

typedef struct
{
  
} MsgTargetPolarLong;

enum
{
  ID_AUTOLIV_REPORT_1              =   0x380,
  ID_AUTOLIV_REPORT_2              =   0x381,

};

}
#endif 
