#ifndef __PLATFORM_LIBEVENT_TCP_LINK_H__
#define __PLATFORM_LIBEVENT_TCP_LINK_H__


#include "snp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

void *libevent_base_create(void);

void libevent_tcp_client_link_create(void *base, struct SNP *snp, char *ip, uint16_t port);

void libevent_tcp_server_link_create(void *base, struct SNP *snp, uint16_t port);

void libevent_tcp_link_exec(void *base, struct SNP *snp);


#ifdef __cplusplus
}
#endif


#endif // __PLATFORM_LIBEVENT_TCP_LINK_H__
