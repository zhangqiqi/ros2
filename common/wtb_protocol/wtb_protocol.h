#ifndef __COMMON_WTB_PROTOCOL_H__
#define __COMMON_WTB_PROTOCOL_H__


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define WTB_MAGIC (0x5aa55aa5)
#define WTB_PORT (10001)


enum WTB_DATA_TYPE {
    LIDAR_WHEELTEC_N10_DATA_UPLOAD,
};


struct WTB_PACKAGE {
    uint32_t magic;      /**< default WTB_MAGIC */
    uint32_t payload_len;
    uint32_t payload_type;      /**< WTB_DATA_TYPE */
    uint32_t payload_check;

    uint8_t payload[];
};



uint32_t wtb_package_unpack(uint8_t *in_buffer, uint32_t in_size, struct WTB_PACKAGE **out_package);

uint32_t wtb_package_pack(enum WTB_DATA_TYPE type, uint8_t *in_buffer, uint32_t in_size, uint8_t *out_buffer, uint32_t out_size);


#ifdef __cplusplus
}
#endif


#endif //__COMMON_WTB_PROTOCOL_H__

