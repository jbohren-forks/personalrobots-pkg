#ifndef _HOST_NETUTIL_H_
#define _HOST_NETUTIL_H_

#include "build.h"
#include "list.h"
#include "ipcam_packet.h"

#include <sys/socket.h>
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

int wgEthGetLocalMac(const char *ifName, struct sockaddr *macAddr);
int wgIpGetLocalBcast(const char *ifName, struct in_addr *bcast);
int wgIpGetLocalAddr(const char *ifName, struct in_addr *addr);

int wgCmdSocketCreate(const char *ifName, NetHost *localHost);
int wgSocketCreate(const struct in_addr *addr, uint16_t port);
int wgSocketConnect(int s, const IPAddress *ip);
int wgSendUDP(int s, const IPAddress *ip, const void *data, size_t dataSize);
int wgSendUDPBcast(int s, const char *ifName, const void *data, size_t dataSize);

int wgArpAdd(IpCamList *camInfo);
int wgArpDel(IpCamList *camInfo);

int wgWaitForPacket( int s, uint32_t type, size_t pktLen, uint32_t *wait_us );

#define SEC_TO_USEC(sec) (1000*1000*sec)

#ifdef __cplusplus
}
#endif

#endif //_HOST_NETUTIL_H_
