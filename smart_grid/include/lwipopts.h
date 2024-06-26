#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"


/* TCP WND must be at least 16 kb to match TLS record size
   or you will get a warning "altcp_tls: TCP_WND is smaller than the RX decrypion buffer, connection RX might stall!" */
#undef TCP_WND
#define TCP_WND  16384

#define LWIP_ALTCP               1
#define LWIP_ALTCP_TLS           1
#define LWIP_ALTCP_TLS_MBEDTLS   1

#define LWIP_DEBUG 1
#define ALTCP_MBEDTLS_DEBUG      LWIP_DBG_ON
#define ALTCP_MBEDTLS_MEM_DEBUG  LWIP_DBG_ON

#define MQTT_DEGUB               LWIP_DBG_ON
#define MEMP_NUM_SYS_TIMEOUT     (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)

#define MQTT_OUTPUT_RINGBUF_SIZE 2048 // 512 for mqtts

#endif
