// license

#ifndef _CAMERA_CALIBRATION_FILE_IO_H_
#define _CAMERA_CALIBRATION_FILE_IO_H_

#include <string>

namespace camera_calibration {

bool writeIntrinsicsIni(const std::string& file_name, const std::string& camera_name,
                        int width, int height,
                        const double* K, const double* D = NULL,
                        const double* R = NULL, const double* P = NULL);

bool readIntrinsicsIni(const std::string& file_name, const std::string& camera_name,
                       int &width, int &height,
                       double* K, double* D = NULL, double* R = NULL, double* P = NULL);

bool parseIntrinsicsIni(const std::string& buffer, const std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);


bool writeIntrinsicsYml(const std::string& file_name, const std::string& camera_name,
                        int width, int height,
                        const double* K, const double* D = NULL,
                        const double* R = NULL, const double* P = NULL);

bool readIntrinsicsYml(const std::string& file_name, const std::string& camera_name,
                       int &width, int &height,
                       double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration

#endif
