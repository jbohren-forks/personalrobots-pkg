#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <net/if.h>
#include <unistd.h>
#include <net/if_arp.h>

#include "list.h"
#include "host_netutil.h"


/**
 * Add a 'permanent' ARP to IP mapping in the system ARP table for one camera.
 * Since the cameras do not support ARP, this step is necessary so that the host
 * can find them.
 *
 * @warning Under Linux, this function requires superuser privileges (or CAP_NET_ADMIN)
 *
 * @param camInfo		An IpCamList element that describes the IP and MAC of the camera
 *
 * @return	Returns 0 for success, -1 with errno set for failure.
 */
int wgArpAdd(IpCamList *camInfo) {
	// Create new permanent mapping in the system ARP table
	struct arpreq arp;
	int s;

	// Create a dummy socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("pr2ArpAdd can't create socket");
		return -1;
	}

	debug("Registering ARP info for S/N %d \n", camInfo->serial);

	// Populate the arpreq structure for this mapping
	((struct sockaddr_in*)&arp.arp_pa)->sin_family = AF_INET;
	memcpy(&((struct sockaddr_in*)&arp.arp_pa)->sin_addr, &camInfo->ip, sizeof(struct in_addr));

	// This is a 'permanent' mapping; it will not time out (but will be cleared by a reboot)
	arp.arp_flags = ATF_PERM;

	arp.arp_ha.sa_family = ARPHRD_ETHER;
	memcpy(&arp.arp_ha.sa_data, camInfo->mac, 6);

	strncpy(arp.arp_dev, camInfo->ifName, sizeof(arp.arp_dev));

	if( ioctl(s, SIOCSARP, &arp) == -1 ) {
		perror("Warning, was unable to create ARP entry (are you root?)");
		return -1;
	} else {
		debug("Camera %u successfully configured\n", camInfo->serial);
	}
	return 0;
}


/**
 * Remove an to IP mapping from the system ARP table for one camera.
 * This function can be used to prevent cluttering the ARP table with unused 'permanent' mappings.
 *
 * @warning Under Linux, this function requires superuser privileges (or CAP_NET_ADMIN)
 *
 * @param camInfo		An IpCamList element that describes the IP and MAC of the camera
 *
 * @return	Returns 0 for success, -1 with errno set for failure.
 */
int wgArpDel(IpCamList *camInfo) {
	// Create new permanent mapping in the system ARP table
	struct arpreq arp;
	int s;

	// Create a dummy socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("pr2ArpDel can't create socket");
		return -1;
	}

	debug("Removing ARP info for S/N %d \n", camInfo->serial);

	// Populate the arpreq structure for this mapping
	((struct sockaddr_in*)&arp.arp_pa)->sin_family = AF_INET;
	memcpy(&((struct sockaddr_in*)&arp.arp_pa)->sin_addr, &camInfo->ip, sizeof(struct in_addr));

	// No flags required for removal
	arp.arp_flags = 0;

	arp.arp_ha.sa_family = ARPHRD_ETHER;
	memcpy(&arp.arp_ha.sa_data, camInfo->mac, 6);

	strncpy(arp.arp_dev, camInfo->ifName, sizeof(arp.arp_dev));

	// Make the request to the kernel
	if( ioctl(s, SIOCDARP, &arp) == -1 ) {
		perror("Warning, was unable to remove ARP entry");
		return -1;
	} else {
		debug("Camera %u successfully removed from ARP table\n", camInfo->serial);
	}

	return 0;
}


/**
 * Utility function to retrieve the MAC address asssociated with
 * a specified Ethernet interface name.
 *
 * @param ifName A null-terminated string containing the name of the Ethernet address (e.g., eth0)
 * @param macAddr A sockaddr structure to contain the MAC
 *
 * @return Returns 0 if successful, -1 with errno set otherwise
 */
int wgEthGetLocalMac(const char *ifName, struct sockaddr *macAddr) {
	struct ifreq ifr;
	int s;

	// Create a dummy socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("wgEthGetLocalMac can't create socket");
		return -1;
	}

	// Initialize the ifreq structure with the interface name
	strncpy(ifr.ifr_name,ifName,sizeof(ifr.ifr_name)-1);
	ifr.ifr_name[sizeof(ifr.ifr_name)-1]='\0';

	// Use socket ioctl to retrieve the HW (MAC) address of this interface
	if( ioctl(s, SIOCGIFHWADDR, &ifr) == -1 ) {
		perror("wgEthGetLocalMac ioctl failed");
		return -1;
	}

	// Transfer address from ifreq struct to output pointer
	memcpy(macAddr, &ifr.ifr_addr, sizeof(struct sockaddr));

	close(s);
	return 0;
}


/**
 * Utility function to retrieve the broadcast IPv4 address asssociated with
 * a specified Ethernet interface name.
 *
 * @param ifName A null-terminated string containing the name of the Ethernet address (e.g., eth0)
 * @param macAddr A in_addr structure to contain the broadcast IP
 *
 * @return Returns 0 if successful, -1 with errno set otherwise
 */
int wgIpGetLocalBcast(const char *ifName, struct in_addr *bcast) {
	struct ifreq ifr;
	int s;

	// Create a dummy socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("wgIpGetLocalBcast can't create socket");
		return -1;
	}

	// Initialize the ifreq structure
	strncpy(ifr.ifr_name,ifName,sizeof(ifr.ifr_name)-1);
	ifr.ifr_name[sizeof(ifr.ifr_name)-1]='\0';

	// Use socket ioctl to get broadcast address for this interface
	if( ioctl(s,SIOCGIFBRDADDR , &ifr) == -1 ) {
		perror("wgIpGetLocalBcast ioctl failed");
		return -1;
	}

	// Requires some fancy casting because the IP portion of a sockaddr_in is stored
	// after the port (which we don't need) in the struct
	memcpy(&(bcast->s_addr), &((struct sockaddr_in *)(&ifr.ifr_broadaddr))->sin_addr, sizeof(struct in_addr));

	close(s);
	return 0;
}

/**
 * Utility function to retrieve the local IPv4 address asssociated with
 * a specified Ethernet interface name.
 *
 * @param ifName A null-terminated string containing the name of the Ethernet address (e.g., eth0)
 * @param macAddr A in_addr structure to contain the local interface IP
 *
 * @return Returns 0 if successful, -1 with errno set otherwise
 */
int wgIpGetLocalAddr(const char *ifName, struct in_addr *addr) {
	struct ifreq ifr;
	int s;

	// Create a dummy socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		perror("wgIpGetLocalAddr can't create socket");
		return -1;
	}

	// Initialize the ifreq structure
	strncpy(ifr.ifr_name,ifName,sizeof(ifr.ifr_name)-1);
	ifr.ifr_name[sizeof(ifr.ifr_name)-1]='\0';

	// Use socket ioctl to get the IP address for this interface
	if( ioctl(s,SIOCGIFADDR , &ifr) == -1 ) {
		perror("wgIpGetLocalAddr ioctl failed");
		return -1;
	}

	// Requires some fancy casting because the IP portion of a sockaddr_in after the port (which we don't need) in the struct
	memcpy(&(addr->s_addr), &((struct sockaddr_in *)(&ifr.ifr_broadaddr))->sin_addr, sizeof(struct in_addr));
	close(s);

	return 0;
}


/**
 * Utility function to create a UDP socket and bind it to a specified address & port.
 *
 * @param addr The host IP address to bind to.
 * @param port The host UDP port to bind to. Host byte order. Port of 0 causes bind() to assign an ephemeral port.
 *
 * @return Returns the bound socket if successful, -1 with errno set otherwise
 */
int wgSocketCreate(const struct in_addr *addr, uint16_t port) {

	// Create a UDP socket for communicating with the network
	int s;
	if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1 ) {
		perror("wgSocketCreate can't create socket");
		return -1;
	}

	// Define the address we'll be listening on
	struct sockaddr_in si_host;
	memset( (uint8_t*)&si_host, 0, sizeof(si_host) );
	si_host.sin_family = AF_INET;								// This is an INET address
	memcpy(&si_host.sin_addr, addr, sizeof(struct in_addr));	// Bind only to this address
	si_host.sin_port = htons(port);								// If port==0,bind chooses an ephemeral source port


	// Bind the socket to the requested port
	if( bind(s, (struct sockaddr *)&si_host, sizeof(si_host)) == -1 ) {
		perror("wgSocketCreate unable to bind");
		return -1;
	}

	// Setting the broadcast flag allows us to transmit to the broadcast address
	// from this socket if we want to. Some platforms do not allow unprivileged
	// users to set this flag, but Linux does.
	int flags = 1;
	if( setsockopt(s, SOL_SOCKET,SO_BROADCAST, &flags, sizeof(flags)) == -1) {
		perror("wgSocketCreate unable to set broadcast socket option");
		return -1;
	}

	return s;
}


/**
 * Utility wrapper to 'connect' a datagram socket to a specific remote host.
 * Once connected, the socket can only receive datagrams from that host.
 *
 * @param s		The open and bound local socket to connect.
 * @param ip 	The remote IP address to connect to.
 *
 * @return 		Returns the result from the connect() system call.
 */
int wgSocketConnect(int s, const IPAddress *ip) {

	struct sockaddr_in camIP;

	camIP.sin_family = AF_INET;
	camIP.sin_port = 0;			// Unused by connect
	camIP.sin_addr.s_addr=*ip;

	if( connect(s, (struct sockaddr *)&camIP, sizeof(camIP)) == -1 ) {
		perror("Could not connect datagram socket");
		return -1;
	}

	return 0;
}


/**
 * Creates and binds a new command packet socket for communicating to a camera. Will always bind to an
 * ephemeral local port number.
 *
 * @param ifName Interface name to bind to. Null terminated string (e.g., "eth0")
 * @param localHost Optional pointer to a structure to receive the local host (MAC/IP/Port) information
 * @return Returns the socket if successful, -1 otherwise
 */
int wgCmdSocketCreate(const char *ifName, NetHost *localHost) {
	// Identify the IP address of the requested interface
	struct in_addr host_addr;
	wgIpGetLocalAddr(ifName, &host_addr);

	// Create & bind a new listening socket, s
	// We specify a port of 0 to have bind choose an available host port
	int s;
	if( (s=wgSocketCreate(&host_addr, 0)) == -1 ) {
		perror("Unable to create socket");
		return -1;
	}

	if(localHost != NULL) {
		struct sockaddr_in socketAddr;
		socklen_t socketAddrSize = sizeof(socketAddr);
		if( getsockname(s, (struct sockaddr *)&socketAddr, &socketAddrSize) == -1) {
			perror("wgSocketToNetHost Could not get socket name");
			return -1;
		}

		struct sockaddr macAddr;
		if( wgEthGetLocalMac(ifName, &macAddr) == -1) {
			return -1;
		}

		memcpy(localHost->mac, macAddr.sa_data, sizeof(localHost->mac));
		localHost->addr = socketAddr.sin_addr.s_addr;
		localHost->port = socketAddr.sin_port;
	}

	return s;
}

/**
 * Utility function to send 'dataSize' bytes of 'data' to remote address 'ip'.
 * This function always sends to the WG_CAMCMD_PORT port.
 *
 * @param s 		Bound socket to send on
 * @param ip IPv4 	Address of remote camera to send to
 * @param data		Array of at least dataSize bytes, containing payload to send
 * @param dataSize	Size of payload to send, in bytes
 *
 * @return Returns 0 if successful. -1 with errno set otherwise.
 * Caller is responsible for closing the socket when finished.
 */
int wgSendUDP(int s, const IPAddress *ip, const void *data, size_t dataSize) {
	// Create and initialize a sockaddr_in structure to hold remote port & IP
	struct sockaddr_in si_cam;
	memset( (uint8_t *)&si_cam, 0, sizeof(si_cam) );
	si_cam.sin_family = AF_INET;
	si_cam.sin_port = htons(WG_CAMCMD_PORT);
	si_cam.sin_addr.s_addr = *ip;

	// Send the datagram
	if( sendto(s, data, dataSize, 0, (struct sockaddr*)&si_cam, sizeof(si_cam)) == -1 ) {
		perror("wgSendUDP unable to send packet");
		close(s);
		return -1;
	}
	return 0;
}

/**
 * Utility function that wraps wgSendUDP to send a packet to the broadcast address on the
 * specified interface.
 *
 * @param s 		Bound socket to send on
 * @param ifName 	Name of interface socket is bound on. (Null terminated string, e.g., "eth0")
 * @param data			Array of at least dataSize bytes, containing payload to send
 * @param dataSize	Size of payload to send, in bytes
 *
 * @return Returns 0 if successful. -1 with errno set otherwise.
 * Caller is responsible for closing the socket when finished.
 */
int wgSendUDPBcast(int s, const char *ifName, const void *data, size_t dataSize) {
	// Identify the broadcast address on the specified interface
	struct in_addr bcastIP;
	wgIpGetLocalBcast(ifName, &bcastIP);

	// Use wgSendUDP to send the broadcast packet
	return wgSendUDP(s, &bcastIP.s_addr, data, dataSize);
}


/**
 * Waits for a specified amount of time for a PR2 packet that matches the specified
 * length and type criteria.
 *
 *	On return, the wait_us argument is updated to reflect the amount of time still remaining
 *	in the original timeout. This can be useful when calling wgWaitForPacket() in a loop.
 *
 * @param s		The datagram socket to listen on. It must be opened, bound, and connected.
 * @param type	The PR2 packet type to listen for. Packets that do not match this type will
 * 				be discarded
 * @param pktLen The length of PR2 packet to listen for. Packets that do not match this length
 * 				will be discarded.
 * @param wait_us The duration of time to wait before timing out. Is adjusted upon return to
 * 					reflect actual time remaning on the timeout.
 *
 * @return Returns -1 with errno set for system call failures. 0 otherwise. If wait_us is set
 * 			to zero, then the wait has timed out.
 */
int wgWaitForPacket( int s, uint32_t type, size_t pktLen, uint32_t *wait_us ) {
	// Convert wait_us argument into a struct timeval
	struct timeval timeout;
	timeout.tv_sec = *wait_us / 1000000UL;
	timeout.tv_usec = *wait_us % 1000000UL;

	// We have been asked to wait wait_us microseconds for a response; compute the time
	// at which the timeout will expire and store it into "timeout"
	struct timeval timestarted;
	struct timeval timenow;
	gettimeofday(&timestarted, NULL);
	gettimeofday(&timenow, NULL);
	timeradd( &timeout, &timestarted, &timeout );

	struct timeval looptimeout;
	fd_set set;
	while( timercmp( &timeout, &timenow, >= ) ) {
		// Since we could receive multiple packets, we need to decrease the timeout
		// to select() as we go. (Multiple packets should be an unlikely event, but
		// UDP provides no guarantees)
		timersub(&timeout, &timestarted, &looptimeout);

		FD_ZERO(&set);
		FD_SET(s, &set);

		// Wait for either a packet to be received or for timeout
		if( select(s+1, &set, NULL, NULL, &looptimeout) == -1 ) {
			perror("wgWaitForPacket select failed");
			return -1;
		}

		// If we received a packet
		if( FD_ISSET(s, &set) ) {
			PacketGeneric gPkt;
			int r;
			// Inspect the packet in the buffer without removing it
			if( (r=recvfrom( s, &gPkt, sizeof(PacketGeneric), MSG_PEEK|MSG_TRUNC, NULL, NULL ))  == -1 ) {
				perror("wgWaitForPacket unable to receive from socket");
				return -1;
			}

			// All valid WG command packets have magic_no == WG_MAGIC NO
			// We also know the minimum packet size we're looking for
			// So we can drop short or invalid packets at this stage
			if( ((unsigned int) r < pktLen) ||
						gPkt.magic_no != htonl(WG_MAGIC_NO) ||
						gPkt.type != htonl(type) ) {
				debug("Dropping packet with magic #%08X, type 0x%02X (looking for 0x%02X), length %d (looking for %d)\n", ntohl(gPkt.magic_no), ntohl(gPkt.type), type, r, pktLen);
				// Pull it out of the buffer (we used MSG_PEEK before, so it's still in there)
				if( recvfrom( s, &gPkt, sizeof(PacketGeneric), 0, NULL, NULL ) == -1 ) {
					perror("wgWaitForPacket unable to receive from socket");
					return -1;
				}
			} else {	// The packet appears to be valid and correct
				// Compute the amount of time left on the timeout in case the calling function
				// decides this is not the packet we're looking for
				struct timeval timeleft;
				gettimeofday(&timenow, NULL);
				timersub(&timeout, &timenow, &timeleft);

				*wait_us = timeleft.tv_usec+timeleft.tv_sec*1000000UL;
				return 0;
			}

		}
		gettimeofday(&timenow, NULL);
	}
	// If we reach here, we've timed out
	*wait_us = 0;
	return 0;
}
