#include "wtb_protocol.h"

#include <stddef.h>


static uint32_t wtb_checksum(uint8_t *data, uint32_t len)
{
    uint32_t checksum = 0;

    while (len-- > 0)
    {
        checksum += data[len];
    }

    return checksum;
}


uint32_t wtb_package_unpack(uint8_t *in_buffer, uint32_t in_size, struct WTB_PACKAGE **out_package)
{
    struct WTB_PACKAGE *package = NULL;

    uint32_t ret_size = 0;

    while (ret_size < in_size)
    {
        package = (struct WTB_PACKAGE *)(in_buffer + ret_size);

        if (WTB_MAGIC != package->magic)
        {
            ret_size++;
            continue;
        }

        if ((in_size - ret_size) < (sizeof(struct WTB_PACKAGE) + package->payload_len))
        {
            break;
        }

        if (package->payload_check != wtb_checksum(package->payload, package->payload_len))
        {
            ret_size++;
            continue;
        }

        ret_size += sizeof(struct WTB_PACKAGE) + package->payload_check;
        *out_package = package;
        break;
    }

    return ret_size;
}


uint32_t wtb_package_pack(enum WTB_DATA_TYPE type, uint8_t *in_buffer, uint32_t in_size, uint8_t *out_buffer, uint32_t out_size)
{

}
