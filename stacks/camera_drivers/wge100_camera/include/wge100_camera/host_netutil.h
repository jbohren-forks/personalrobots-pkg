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

int wge100EthGetLocalMac(const char *ifName, struct sockaddr *macAddr);
int wge100IpGetLocalNetmask(const char *ifName, struct in_addr *bcast);
int wge100IpGetLocalBcast(const char *ifName, struct in_addr *bcast);
int wge100IpGetLocalAddr(const char *ifName, struct in_addr *addr);

int wge100CmdSocketCreate(const char *ifName, NetHost *localHost);
int wge100SocketCreate(const struct in_addr *addr, uint16_t port);
int wge100SocketConnect(int s, const IPAddress *ip);
int wge100SendUDP(int s, const IPAddress *ip, const void *data, size_t dataSize);
int wge100SendUDPBcast(int s, const char *ifName, const void *data, size_t dataSize);

int wge100ArpAdd(IpCamList *camInfo);
int wge100ArpDel(IpCamList *camInfo);

int wge100WaitForPacket( int *s, int nums, uint32_t type, size_t pktLen, uint32_t *wait_us );

#define SEC_TO_USEC(sec) (1000*1000*sec)

#ifdef __cplusplus
}
#endif

#endif //_HOST_NETUTIL_H_
