#include "camera_calibration/pinhole.h"
#include "camera_calibration/file_io.h"

#include <boost/algorithm/string/predicate.hpp>

namespace camera_calibration {

PinholeCameraModel::PinholeCameraModel()
{
}

bool PinholeCameraModel::load(const std::string& file_name)
{
  if (boost::iends_with(file_name, ".ini"))
    return readIntrinsicsIni(file_name, camera_name, image_width_, image_height_, K, D);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return readIntrinsicsYml(file_name, camera_name, image_width_, image_height_, K, D);

  return false;
}

bool PinholeCameraModel::save(const std::string& file_name)
{
  if (boost::iends_with(file_name, ".ini"))
    return writeIntrinsicsIni(file_name, camera_name, image_width_, image_height_, K, D);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return writeIntrinsicsYml(file_name, camera_name, image_width_, image_height_, K, D);

  return false;
}

bool PinholeCameraModel::parse(const std::string& buffer, const std::string& format)
{
  if (format != "ini")
    return false;
  
  return parseIntrinsicsIni(buffer, camera_name, image_width_, image_height_, K, D);
}

} //namespace camera_calibration
