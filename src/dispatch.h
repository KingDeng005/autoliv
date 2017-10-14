#ifndef _DISPATCH_H
#define _DISPATCH_H
#include <stdint.h>

namespace octopus
{
typedef struct
{
  uint8_t  MD1:8;
  uint8_t  MD2:8;
  uint8_t  MD3:8;
  uint8_t  MD4:8;
  uint8_t  MD5:8;
  uint8_t  SenNum_MD1:4;
  uint8_t  MD1_Msg_cntr:4;
  uint8_t  MD6:8;
  uint8_t  MD7:8;
} MsgObjectReport1;

typedef struct
{
  uint8_t  MD8:8;
  uint8_t  MD9:8;
  uint8_t  MD10:8;
  uint8_t  MD11:8;
  uint8_t  MD12:8;
  uint8_t  SenNum_MD2:4;
  uint8_t  MD2_Msg_cntr:4;
  uint8_t  MD14:8;
  uint8_t  MD13:8;
} MsgObjectReport2;


enum
{
  ID_FLR_MEASURE_DATA_1_REPORT            =   0x380,
  ID_FLR_MEASURE_DATA_2_REPORT            =   0x380,
};

}
#endif 
