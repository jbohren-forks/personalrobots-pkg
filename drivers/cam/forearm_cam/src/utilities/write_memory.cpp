#include "pr2lib.h"
#include "host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 5) {
    printf("Usage: %s <Interface> <IP address> <Serial number> <file>\n", argv[0]);
    return 0;
  }
  char* if_name = argv[1];
  char* ip_address = argv[2];
  int sn = atoi(argv[3]);
  char* filename = argv[4];

  // Create a new IpCamList to hold the camera list
  IpCamList camList;
  pr2CamListInit(&camList);

  // Discover any connected cameras, wait for 0.5 second for replies
  if( pr2Discover(if_name, &camList, SEC_TO_USEC(0.5)) == -1) {
    printf("Discover error\n");
    return -1;
  }

  if (pr2CamListNumEntries(&camList) == 0) {
    printf("No cameras found\n");
    return -1;
  }

  // Open camera with requested serial number
  int index = pr2CamListFind(&camList, sn);
  if (index == -1) {
    printf("Couldn't find camera with S/N %i\n", sn);
    return -1;
  }
  IpCamList* camera = pr2CamListGetEntry(&camList, index);

  // Configure the camera with its IP address, wait up to 500ms for completion
  int retval = pr2Configure(camera, ip_address, SEC_TO_USEC(0.5));
  if (retval != 0) {
    if (retval == ERR_CONFIG_ARPFAIL) {
      printf("Unable to create ARP entry (are you root?), continuing anyway\n");
    } else {
      printf("IP address configuration failed\n");
      return -1;
    }
  }

  // Read data from file
  // FIXME: Currently actually filling buffer with test data to demonstrate
  //        flash read/write bug
  uint8_t buffer[FLASH_PAGE_SIZE];// = {0};
  //memset(buffer, 'B', FLASH_PAGE_SIZE);
  memset(buffer, 'A', 268);
  memset(buffer + 268, 'B', FLASH_PAGE_SIZE - 268);
#if 0
  FILE* file = fopen(filename, "rb");
  assert(file);
  fseek(file, 0, SEEK_END);
  size_t size = ftell(file);
  if (size > FLASH_PAGE_SIZE) {
    printf("File is too big!\n");
    return -1;
  }
  fseek(file, 0, SEEK_SET);
  fread(buffer, 1, size, file);
  fclose(file);
#endif
  printf("Data to write:\n****************\n");
  fwrite(buffer, 1, FLASH_PAGE_SIZE, stdout);
  printf("\n****************\n");

  // Write data to flash memory
  static const int PAGE_NUMBER = 4092;
  if (pr2FlashWrite(camera, PAGE_NUMBER, buffer) != 0) {
    printf("Flash write error\n");
    return -1;
  }
  printf("Wrote to page %i, verifying...\n", PAGE_NUMBER);

  // Verify
  uint8_t data[FLASH_PAGE_SIZE];
  if (pr2FlashRead(camera, PAGE_NUMBER, data) != 0) {
    printf("Flash read error, couldn't verify\n");
    return -1;
  }
  printf("Data read back:\n****************\n");
  fwrite(data, 1, FLASH_PAGE_SIZE, stdout);
  printf("\n****************\n");
  
  if (memcmp(buffer, data, FLASH_PAGE_SIZE) != 0) {
    printf("Data read back does not match data written!\n");
    return -1;
  }

  return 0;
}
