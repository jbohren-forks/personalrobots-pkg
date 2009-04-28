#ifndef _PR2LIB_H_
#define _PR2LIB_H_
#include <stdint.h>
#include "list.h"
#include "ipcam_packet.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Firmware version number
 */
#define PR2LIB_VERSION_MAJOR 0x01
#define PR2LIB_VERSION_MINOR 0x04
#define PR2LIB_VERSION ((PR2LIB_VERSION_MAJOR <<8) | PR2LIB_VERSION_MINOR )


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


/*
 * The pr2FrameInfo structure is returned to the frame handler
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
} pr2FrameInfo;



int pr2LibVersion( void );

int pr2Discover(const char *ifName, IpCamList *ipCamList, unsigned wait_us);
int pr2Configure( IpCamList *camInfo, const char *ipAddress, unsigned wait_us);
int pr2StartVid( const IpCamList *camInfo, const uint8_t mac[6], const char *ipAddress, unsigned short port );
int pr2StopVid( const IpCamList *camInfo );
int pr2Reset( IpCamList *camInfo );
int pr2GetTimer( const IpCamList *camInfo, uint64_t *time_us );
int pr2FlashRead( const IpCamList *camInfo, uint32_t address, uint8_t *pageDataOut );
int pr2FlashWrite( const IpCamList *camInfo, uint32_t address, const uint8_t *pageDataIn );
int pr2SensorRead( const IpCamList *camInfo, uint8_t reg, uint16_t *data );
int pr2SensorWrite( const IpCamList *camInfo, uint8_t reg, uint16_t data );
int pr2ConfigureBoard( const IpCamList *camInfo, uint32_t serial, MACAddress *mac );
int pr2TriggerControl( const IpCamList *camInfo, uint32_t triggerType );
int pr2SensorSelect( const IpCamList *camInfo, uint8_t index, uint32_t reg );
int pr2ImagerSetRes( const IpCamList *camInfo, uint16_t horizontal, uint16_t vertical );
int pr2ImagerModeSelect( const IpCamList *camInfo, uint32_t mode );

/// A FrameHandler function returns zero to continue to receive data, non-zero otherwise
typedef int (*FrameHandler)(pr2FrameInfo *frame_info, void *userData);
int pr2VidReceive( const char *ifName, uint16_t port, size_t height, size_t width, FrameHandler frameHandler, void *userData );

#define CONFIG_PRODUCT_ID 6805018
#define ERR_TIMEOUT 100
#define ERR_CONFIG_ARPFAIL 200

#ifdef __cplusplus
}
#endif

#endif // _PR2LIB_H_
