#ifndef _FCAMLIB_H_
#define _FCAMLIB_H_
#include <stdint.h>
#include "list.h"
#include "ipcam_packet.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Firmware version number
 */
#define FCAMLIB_VERSION_MAJOR 0x01
#define FCAMLIB_VERSION_MINOR 0x05
#define FCAMLIB_VERSION ((FCAMLIB_VERSION_MAJOR <<8) | FCAMLIB_VERSION_MINOR )


/*
 * Trigger Modes
 */

#define TRIG_STATE_INTERNAL 0
#define TRIG_STATE_EXTERNAL 1

/*
 * Imager Modes
 */
#define MT9VMODE_752x480x15b1 		0
#define MT9VMODE_752x480x12_5b1 	1
#define MT9VMODE_640x480x30b1		2
#define MT9VMODE_640x480x25b1		3
#define MT9VMODE_640x480x15b1		4
#define MT9VMODE_640x480x12_5b1		5
#define MT9VMODE_320x240x60b2		6
#define MT9VMODE_320x240x50b2		7
#define MT9VMODE_320x240x30b2		8
#define MT9VMODE_320x240x25b2 		9
#define MT9V_NUM_MODES 10

struct MT9VMode {
  char *name;
  size_t width;
  size_t height;
  double fps;
  uint16_t hblank;
  uint16_t vblank;
};

extern const struct MT9VMode MT9VModes[MT9V_NUM_MODES];

/*
 * The fcamFrameInfo structure is returned to the frame handler
 */  
typedef struct
{
  size_t width;
  size_t height;

  uint32_t frame_number;

  size_t lastMissingLine;
  size_t missingLines;
  int shortFrame;

  uint8_t *frameData;
  PacketEOF *eofInfo;

  struct timeval startTime;
} fcamFrameInfo;



int fcamLibVersion( void );

int fcamDiscover(const char *ifName, IpCamList *ipCamList, const char *ipAddress, unsigned wait_us);
int fcamConfigure( IpCamList *camInfo, const char *ipAddress, unsigned wait_us);
int fcamStartVid( const IpCamList *camInfo, const uint8_t mac[6], const char *ipAddress, unsigned short port );
int fcamStopVid( const IpCamList *camInfo );
int fcamReset( IpCamList *camInfo );
int fcamReconfigureFPGA( IpCamList *camInfo );
int fcamGetTimer( const IpCamList *camInfo, uint64_t *time_us );
int fcamReliableFlashRead( const IpCamList *camInfo, uint32_t address, uint8_t *pageDataOut, int *retries );
int fcamFlashRead( const IpCamList *camInfo, uint32_t address, uint8_t *pageDataOut );
int fcamReliableFlashWrite( const IpCamList *camInfo, uint32_t address, const uint8_t *pageDataIn, int *retries );
int fcamFlashWrite( const IpCamList *camInfo, uint32_t address, const uint8_t *pageDataIn );
int fcamReliableSensorRead( const IpCamList *camInfo, uint8_t reg, uint16_t *data, int *retries );
int fcamSensorRead( const IpCamList *camInfo, uint8_t reg, uint16_t *data );
int fcamReliableSensorWrite( const IpCamList *camInfo, uint8_t reg, uint16_t data, int *retries );
int fcamSensorWrite( const IpCamList *camInfo, uint8_t reg, uint16_t data );
int fcamConfigureBoard( const IpCamList *camInfo, uint32_t serial, MACAddress *mac );
int fcamTriggerControl( const IpCamList *camInfo, uint32_t triggerType );
int fcamSensorSelect( const IpCamList *camInfo, uint8_t index, uint32_t reg );
int fcamImagerSetRes( const IpCamList *camInfo, uint16_t horizontal, uint16_t vertical );
int fcamImagerModeSelect( const IpCamList *camInfo, uint32_t mode );

/// A FrameHandler function returns zero to continue to receive data, non-zero otherwise
typedef int (*FrameHandler)(fcamFrameInfo *frame_info, void *userData);
int fcamVidReceive( const char *ifName, uint16_t port, size_t height, size_t width, FrameHandler frameHandler, void *userData );

#define CONFIG_PRODUCT_ID 6805018
#define ERR_TIMEOUT 100
#define ERR_CONFIG_ARPFAIL 200

#ifdef __cplusplus
}
#endif

#endif // _FCAMLIB_H_
