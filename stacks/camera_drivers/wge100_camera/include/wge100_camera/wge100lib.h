#ifndef _WGE100LIB_H_
#define _WGE100LIB_H_
#include <stdint.h>
#include "list.h"
#include "ipcam_packet.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Firmware version number
 */
#define WGE100LIB_VERSION_MAJOR 0x01
#define WGE100LIB_VERSION_MINOR 0x05
#define WGE100LIB_VERSION ((WGE100LIB_VERSION_MAJOR <<8) | WGE100LIB_VERSION_MINOR )


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
 * The wge100FrameInfo structure is returned to the frame handler
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
} wge100FrameInfo;



int wge100LibVersion( void );

int wge100FindByUrl(const char *url, IpCamList *camera, unsigned wait_us, const char **errmsg);
int wge100Discover(const char *ifName, IpCamList *ipCamList, const char *ipAddress, unsigned wait_us);
int wge100Configure( IpCamList *camInfo, const char *ipAddress, unsigned wait_us);
int wge100StartVid( const IpCamList *camInfo, const uint8_t mac[6], const char *ipAddress, uint16_t port );
int wge100StopVid( const IpCamList *camInfo );
int wge100Reset( IpCamList *camInfo );
int wge100ReconfigureFPGA( IpCamList *camInfo );
int wge100GetTimer( const IpCamList *camInfo, uint64_t *time_us );
int wge100ReliableFlashRead( const IpCamList *camInfo, uint32_t address, uint8_t *pageDataOut, int *retries );
int wge100FlashRead( const IpCamList *camInfo, uint32_t address, uint8_t *pageDataOut );
int wge100ReliableFlashWrite( const IpCamList *camInfo, uint32_t address, const uint8_t *pageDataIn, int *retries );
int wge100FlashWrite( const IpCamList *camInfo, uint32_t address, const uint8_t *pageDataIn );
int wge100ReliableSensorRead( const IpCamList *camInfo, uint8_t reg, uint16_t *data, int *retries );
int wge100SensorRead( const IpCamList *camInfo, uint8_t reg, uint16_t *data );
int wge100ReliableSensorWrite( const IpCamList *camInfo, uint8_t reg, uint16_t data, int *retries );
int wge100SensorWrite( const IpCamList *camInfo, uint8_t reg, uint16_t data );
int wge100ConfigureBoard( const IpCamList *camInfo, uint32_t serial, MACAddress *mac );
int wge100TriggerControl( const IpCamList *camInfo, uint32_t triggerType );
int wge100SensorSelect( const IpCamList *camInfo, uint8_t index, uint32_t reg );
int wge100ImagerSetRes( const IpCamList *camInfo, uint16_t horizontal, uint16_t vertical );
int wge100ImagerModeSelect( const IpCamList *camInfo, uint32_t mode );

/// A FrameHandler function returns zero to continue to receive data, non-zero otherwise
typedef int (*FrameHandler)(wge100FrameInfo *frame_info, void *userData);
int wge100VidReceive( const char *ifName, uint16_t port, size_t height, size_t width, FrameHandler frameHandler, void *userData );
int wge100VidReceiveAuto( IpCamList *camera, size_t height, size_t width, FrameHandler frameHandler, void *userData );

#define CONFIG_PRODUCT_ID 6805018
#define ERR_TIMEOUT 100
#define ERR_CONFIG_ARPFAIL 200

#ifdef __cplusplus
}
#endif

#endif // _WGE100LIB_H_
