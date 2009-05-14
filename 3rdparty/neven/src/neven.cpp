
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>

#include <neven/neven.h>

#include <fd_emb_sdk.h>

#include "RFFspeed_501.i"
#include "RFFstd_501.i"
#include "RFFprec_501.i"

using namespace neven;

FaceDetector::FaceDetector(uint32_t max_width, uint32_t max_height, uint32_t max_faces, ProcessingConfig conf)
{

  btk_SDKCreateParam sdkParam = btk_SDK_defaultParam();
  sdkParam.fpMalloc = malloc;
  sdkParam.fpFree = free;
  sdkParam.maxImageWidth = max_width;
  sdkParam.maxImageHeight = max_height;

  btk_Status status = btk_SDK_create(&sdkParam, &sdk_);

  // make sure everything went well
  if (status != btk_STATUS_OK) {
    throw std::runtime_error("Failed to initialize btk_SDK");
  }

  btk_DCRCreateParam dcrParam = btk_DCR_defaultParam();
  btk_DCR_create( sdk_, &dcrParam, &dcr_ );

  btk_FaceFinderCreateParam fdParam = btk_FaceFinder_defaultParam();

  switch (conf)
  {
  case CONFIG_SPEED:
    fdParam.pModuleParam = RFFspeed_501;
    fdParam.moduleParamSize = sizeof(RFFspeed_501);
    break;
  case CONFIG_STANDARD:
    fdParam.pModuleParam = RFFstd_501;
    fdParam.moduleParamSize = sizeof(RFFstd_501);
    break;
  case CONFIG_PRECISION:
  default:
    fdParam.pModuleParam = RFFprec_501;
    fdParam.moduleParamSize = sizeof(RFFprec_501);
    break;
  }

  fdParam.maxDetectableFaces = max_faces;
  status = btk_FaceFinder_create( sdk_, &fdParam, &fd_ );
  btk_FaceFinder_setRange(fd_, 20, 240/2); /* set eye distance range */

  // make sure everything went welln
  if (status != btk_STATUS_OK) {
    throw std::runtime_error("Failed to initialize btk_FaceFinder");
  }
}

FaceDetector::~FaceDetector()
{
    btk_FaceFinder_close( fd_ );
    btk_DCR_close( dcr_ );
    btk_SDK_close( sdk_ );
}

std::vector<Face> FaceDetector::findFaces(char* img, uint32_t width, uint32_t height)
{
  std::vector<Face> faces;

  btk_DCR_assignGrayByteImage(dcr_, img, width, height);
  
  if (btk_FaceFinder_putDCR(fd_, dcr_) == btk_STATUS_OK) {

    int numberOfFaces = btk_FaceFinder_faces(fd_);

    for (int i = 0; i < numberOfFaces; i++)
    {

      btk_FaceFinder_getDCR(fd_, dcr_);

      btk_Node leftEye, rightEye;

      btk_DCR_getNode(dcr_, 0, &leftEye);
      btk_DCR_getNode(dcr_, 1, &rightEye);

      Face fdata;

      fdata.eyedist = (float)(rightEye.x - leftEye.x) / (1 << 16);
      fdata.midpointx = (float)(rightEye.x + leftEye.x) / (1 << 17);
      fdata.midpointy = (float)(rightEye.y + leftEye.y) / (1 << 17);
      fdata.confidence = (float)btk_DCR_confidence(dcr_) / (1 << 24);

      faces.push_back(fdata);
    }
  }

  return faces;
}
