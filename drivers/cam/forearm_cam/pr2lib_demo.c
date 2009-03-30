/*
 * @file main.c
 *
 */
#include "build.h"
#include "list.h"
#include "pr2lib.h"
#include "ipcam_packet.h"
#include "host_netutil.h"

#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


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

int testHandler(size_t width, size_t height, uint8_t *frameData, PacketEOF *eofInfo, void *userData) {
	if(eofInfo == NULL) {
		printf("\tFrame was missing EOF, no frame information available\n");
		// Optionally do something with the frame data
	} else {
		// Do something with frame header
		printf("New frame #%u (%ux%u). ", eofInfo->header.frame_number, width, height);

		// Do something with I2C values
		printf("I2C: ");

		for(int i=0; i<eofInfo->i2c_valid; i++) {
			printf("%u:%04X ", i, eofInfo->i2c[i]);
		}

		// Do something with EOF time
		uint64_t frame_time_us;
		frame_time_us = ((uint64_t)eofInfo->ticks_hi << 32) + eofInfo->ticks_lo;

		// We want milliseconds from seconds, but to preserve some precision
		// we'll multiply the numerator by 1000 and divide the denominator
		// by 1000.
		frame_time_us *= 1000;
		frame_time_us /= (eofInfo->ticks_per_sec/1000);
		printf("Time: %llu us ", frame_time_us);

		// Check for short packet (video lines were missing)
		if(eofInfo->header.line_number == IMAGER_LINENO_SHRT) {
			printf("(Short)");
		}
		printf("\n");

		// Do something with video data
		char filename[100];
		if(eofInfo->header.line_number == IMAGER_LINENO_SHRT) {
			sprintf(filename, "v%05uS.pgm", eofInfo->header.frame_number);
		} else {
			sprintf(filename, "v%05u.pgm", eofInfo->header.frame_number);
		}

		FILE *myfile = fopen(filename, "wb");
		fprintf(myfile, "P5\n%i %i\n255\n", width, height);
		fwrite(frameData, sizeof(uint8_t), height*width, myfile);
		fclose(myfile);
	}

	// Return 0 to continue receiving data, return non-zero when done
	return 0;	// Receive forever
}

int main(int argc, char **argv) {

	if(argc != 3) {
		printf("Usage %s ifname testip\n", argv[0]);
		exit(1);
	}

	// Create a new IpCamList to hold the camera list
	IpCamList camList;
	pr2CamListInit(&camList);

	// Discover any connected cameras, wait for 0.5 second for replies
	if( pr2Discover(argv[1], &camList, SEC_TO_USEC(0.5)) == -1) {
		printf("Discover error\n");
		return 1;
	}

	if (pr2CamListNumEntries(&camList) == 0) {
		printf("No cameras found\n");
		return 1;
	}

	// Step through all cameras and test functionality
	for(int i=0; i<pr2CamListNumEntries(&camList); i++) {
		IpCamList *myCamera = pr2CamListGetEntry(&camList, i);

		// Configure the camera with the test IP address, wait up to 500ms for completion
		int retval;
		if( (retval=pr2Configure(myCamera, argv[2], SEC_TO_USEC(0.5))) != 0) {
			printf("Configure failed.\n");
			if(retval != ERR_CONFIG_ARPFAIL)
				return 1;
			printf("Must be root to make changes to the ARP table.\n");
		}
		printf("Configured camera #%d, S/N #%u, IP address %s\n", i, myCamera->serial, argv[2]);



		// Request camera time
		uint64_t cameraTimeStart;

		if(pr2GetTimer(myCamera, &cameraTimeStart) != 0) {
			printf("Get start time error\n");
			return 1;
		}
		printf("Camera local time is %llu microseconds\n", cameraTimeStart);


//#define CONFIGURE_OTP
#ifdef CONFIGURE_OTP
#warning WARNING, this programs the OTP portion of the board

		// New MAC address goes here
		struct sockaddr newMac;
		newMac.sa_data[0]=0x00;
		newMac.sa_data[1]=0x24;
		newMac.sa_data[2]=0xCD;
		newMac.sa_data[3]=0x00;
		newMac.sa_data[4]=0x00;
		newMac.sa_data[5]=0x6E;

		// New serial number goes here
		uint32_t newSerial=10;

		puts("WARNING WARNING WARNING");
		puts("ABOUT TO PROGRAM OTP SECTION OF THE BOARD");
		printf("NEW VALUES: Serial = %04u  :  MAC = %02X:%02X:%02X:%02X:%02X:%02X\n", newSerial,
			(unsigned char)newMac.sa_data[0],
			(unsigned char)newMac.sa_data[1],
			(unsigned char)newMac.sa_data[2],
			(unsigned char)newMac.sa_data[3],
			(unsigned char)newMac.sa_data[4],
			(unsigned char)newMac.sa_data[5]);

		puts("ENTER '#' to continue");

		if(getchar() != '#') {
			printf("Abort\n");
			exit(1);
		}

		if (pr2ConfigureBoard(myCamera, newSerial, &newMac) != 0) {
			printf("Configure board error\n");
			return 1;
		}

		printf("Configure done\n");


#endif

		// Start local timer
		struct timeval localStart;
		gettimeofday(&localStart, NULL);
		uint64_t localTimeStart = SEC_TO_USEC(localStart.tv_sec)+localStart.tv_usec;

//#define FLASH_TEST
#ifdef FLASH_TEST
#define FLASH_TEST_PAGE 4092
#warning Including Flash Test

		// Read last page from Dataflash
		printf("Flash testing, do not interrupt\n");
		uint8_t data[FLASH_PAGE_SIZE];
		if( pr2FlashRead(myCamera, FLASH_TEST_PAGE, data) != 0 ) {
			printf("Flash read error.\n");
			return 1;
		}

		// Invert it
		uint8_t newData[FLASH_PAGE_SIZE];

		for(int i=0; i<FLASH_PAGE_SIZE; i++) {
			newData[i] = ~data[i];
		}

		// Write it back
		if( pr2FlashWrite(myCamera, FLASH_TEST_PAGE, newData) != 0 ) {
			printf("Flash write error.\n");
			return 1;
		}

		memset(newData, 0, sizeof(newData));

		// Read it again
		if( pr2FlashRead(myCamera, FLASH_TEST_PAGE, newData) != 0 ) {
			printf("Flash read error.\n");
			return 1;
		}

		// Verify
		for(int i=0; i<FLASH_PAGE_SIZE; i++) {
			if( newData[i] != (uint8_t)~data[i] ) {
				printf("Flash data comparison failed, byte %d: %02x vs %02x\n", i, data[i], (uint8_t)~newData[i]);
			}
		}
		printf("Flash comparison complete\n");

		// And put the original flash page back in place
		if( pr2FlashWrite(myCamera, FLASH_TEST_PAGE, data) != 0 ) {
			printf("Flash write error.\n");
			return 1;
		}
#endif


//#define I2C_TEST
#ifdef I2C_TEST
#warning Including I2C Test

		// Test read all I2C registers
		// Select a video mode
		uint32_t vidMode=MT9VMODE_752x480x15b1;
		printf("Selecting video mode %u\n", vidMode);
		if( pr2ImagerModeSelect( myCamera, vidMode ) != 0) {
			printf("Mode select error.\n");
			return 1;
		}

		uint16_t i2cData;
		printf("All I2C imager registers:\n");
		for(int i=0; i<256; i++) {
			if( pr2SensorRead( myCamera, i, &i2cData ) != 0) {
				printf("Sensor I2C read error.\n");
				return 1;
			}
			printf("%04X%c", i2cData, (i%16==15)?'\n':' ' );
		}




		// Set a different video mode, to show which registers have changed
		// Select a video mode
		vidMode=MT9VMODE_640x480x30b1;
		printf("Selecting video mode %u\n", vidMode);
		if( pr2ImagerModeSelect( myCamera, vidMode ) != 0) {
			printf("Mode select error.\n");
			return 1;
		}

		printf("All I2C imager registers in video mode %u:\n", vidMode);
		for(int i=0; i<256; i++) {
			if( pr2SensorRead( myCamera, i, &i2cData ) != 0 ) {
				printf("Sensor I2C read error.\n");
				return 1;
			}
			printf("%04X%c", i2cData, (i%16==15)?'\n':' ' );
		}

		// Read the sensor lock register (default value is 0xBEEF)
		if( pr2SensorRead( myCamera, 0xFE, &i2cData) != 0) {
			printf("Sensor I2C read error.\n");
			return 1;
		}
		printf("Sensor lock register is 0x%04X\n", i2cData);

		// Write in a different value
		i2cData = 0xDEAD;
		printf("Writing value 0x%04X to sensor lock register\n", i2cData);
		if( pr2SensorWrite( myCamera, 0xFE, i2cData) != 0 ) {
			printf("Sensor I2C write error.\n");
			return 1;
		}

		// Verify
		if( pr2SensorRead( myCamera, 0xFE, &i2cData ) != 0 ) {
			printf("Sensor I2C read error.\n");
			return 1;
		}
		printf("Sensor lock register is 0x%04X\n", i2cData);

		// Write back the default value
		i2cData = 0xBEEF;
		printf("Writing value 0x%04X to sensor lock register\n", i2cData);
		if( pr2SensorWrite( myCamera, 0xFE, i2cData) != 0 ) {
			printf("Sensor I2C write error.\n");
			return 1;
		}

		printf("I2C testing done\n");

#endif

		// We are going to receive the video on this host, so we need our own MAC address
		struct sockaddr localMac;
		if( wgEthGetLocalMac(myCamera->ifName, &localMac) != 0 ) {
			printf("Unable to get local MAC address for interface %s\n", myCamera->ifName);
			return 1;
		}

		struct in_addr localIp;
		// We also need our local IP address
		if( wgIpGetLocalAddr(myCamera->ifName, &localIp) != 0) {
			printf("Unable to get local IP address for interface %s\n", myCamera->ifName);
			return 1;
		}

#define VID_TEST
#ifdef VID_TEST
#warning Including Video Test

		// Select a video mode
		printf("Selecting video mode %u\n", MT9VMODE_640x480x30b1);
		if( pr2ImagerModeSelect( myCamera, MT9VMODE_640x480x30b1 ) != 0) {
			printf("Mode select error.\n");
			return 1;
		}

		// Select four I2C auto-read registers
		uint8_t autoReadRegs[4] = {0x00, 0x01, 0x02, 0x03};
		for(int i=0; i<4; i++) {
			printf("Selecting register address 0x%02X for read position %d\n", autoReadRegs[i], i);
			if( pr2SensorSelect(myCamera, i, autoReadRegs[i]) != 0 ) {
				printf("Sensor I2C select error.\n");
				return 1;
			}
		}

		//Per a request from Paul Karazuba @ Aptina we're increasing the VBLANK for testing
		//if (0 != pr2SensorWrite( myCamera, 0x0A, 0x016E )) {
		//	printf("Failed to change DEBUG register\n");
		//	return 1;
		//}

		printf("Dumping I2C sensor registers:\n");
		for(int i=0; i<256; i++) {
			uint16_t i2cData;
			if( pr2SensorRead( myCamera, i, &i2cData ) != 0 ) {
				printf("Sensor I2C read error.\n");
				return 1;
			}
			printf("%04X%c", i2cData, (i%16==15)?'\n':' ' );
		}


		printf("Press enter to start video streaming\n");
		getchar();

//#define CYGWIN_TEST
#ifdef CYGWIN_TEST
#warning Including CYGWIN Test
		localMac.sa_data[0]=0x00;
		localMac.sa_data[1]=0x16;
		localMac.sa_data[2]=0x41;
		localMac.sa_data[3]=0x57;
		localMac.sa_data[4]=0x05;
		localMac.sa_data[5]=0x11;

		if( pr2StartVid( myCamera, (uint8_t *)&(localMac.sa_data[0]), "192.168.65.1", 9090) != 0 ) {
			printf("Video start error\n");
			return 1;
		}
		//pr2VidReceive( myCamera->ifName, 9090, 480, 640 );
		sleep(1);
#else

//#define EXTTRIG_TEST
#ifdef EXTTRIG_TEST
#warning Including External Trigger Test

		printf("Selecting external trigger\n");
		if ( pr2TriggerControl( myCamera, TRIG_STATE_EXTERNAL ) != 0) {
			printf("Trigger mode select error\n");
			return 1;
		}
#endif

		// Start video; send it to this host on port 9090 (arbitrarily chosen for this demo app)
		if( pr2StartVid( myCamera, (uint8_t *)&(localMac.sa_data[0]), inet_ntoa(localIp), 9090) != 0 ) {
			printf("Video start error\n");
			return 1;
		}

		//pr2VidReceive( myCamera->ifName, 9090, 480, 752 );
		pr2VidReceive( myCamera->ifName, 9090, 480, 640, testHandler, NULL );
		//pr2VidReceive( myCamera->ifName, 9090, 240, 320 );
#endif  //CYGWIN_TEST

#define STOP_TEST
#ifdef STOP_TEST
#warning Including Stop Test
		// Stop video
		if(pr2StopVid(myCamera) != 0 ) {
			printf("Video Stop error.\n");
			return 1;
		}
#endif



		// Request camera time again
		uint64_t cameraTimeStop;
		if( pr2GetTimer(myCamera, &cameraTimeStop) != 0 ) {
			printf("Get stop time error.\n");
			return 1;
		}
		printf("Camera local time is %llu microseconds\n", cameraTimeStop);

		// Stop local timer
		struct timeval localStop;
		gettimeofday(&localStop, NULL);
		uint64_t localTimeStop = SEC_TO_USEC(localStop.tv_sec)+localStop.tv_usec;

		// Compare results
		printf("PC timer reported %llu microseconds elapsed.\n", localTimeStop-localTimeStart);
		printf("Camera timer reported %llu microseconds elapsed.\n", cameraTimeStop-cameraTimeStart);

#endif  //VID_TEST


//#define RESET_TEST
#ifdef RESET_TEST
#warning Including Reset Test

		// Reset camera
		printf("Resetting.\n");
		if( pr2Reset(myCamera) != 0 ) {
			printf("Camera reset error.\n");
			return 1;
		}

		// Wait for reset to complete
		usleep(SEC_TO_USEC(0.3));

		// Wait for network on host side to reestablish link since we're using a direct crossover cable
		sleep(2);

		// Read an I2C register (will timeout since camera is not configured)
		// A timeout is a protocol error and will result in a return value of 1
		uint16_t i2cDummy;
		if( pr2SensorRead( myCamera, 0xFE, &i2cDummy ) == 1 ) {
			printf("Sensor timed out as expected. Camera reset successful.\n");
		} else {
			printf("Error: Camera failed to reset!\n");
		}
#endif
	}
	return 0;
}

