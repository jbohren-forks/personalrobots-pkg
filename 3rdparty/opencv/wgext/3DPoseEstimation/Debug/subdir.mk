################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Cv3DPoseEstimate.cpp \
../Cv3DPoseEstimateDisp.cpp \
../Cv3DPoseEstimateDispSpaceRef.cpp \
../Cv3DPoseEstimateRef.cpp \
../Cv3DPoseEstimateSSE.cpp \
../CvLevMarq3D.cpp \
../CvLevMarqDispSpace.cpp \
../CvMat3X3.cpp \
../CvMatUtils.cpp \
../CvStereoCamModel.cpp \
../CvStereoCamParams.cpp \
../CvTest3DPoseEstimate.cpp \
../CvTestTimer.cpp \
../calib_stereo.cpp 

OBJS += \
./Cv3DPoseEstimate.o \
./Cv3DPoseEstimateDisp.o \
./Cv3DPoseEstimateDispSpaceRef.o \
./Cv3DPoseEstimateRef.o \
./Cv3DPoseEstimateSSE.o \
./CvLevMarq3D.o \
./CvLevMarqDispSpace.o \
./CvMat3X3.o \
./CvMatUtils.o \
./CvStereoCamModel.o \
./CvStereoCamParams.o \
./CvTest3DPoseEstimate.o \
./CvTestTimer.o \
./calib_stereo.o 

CPP_DEPS += \
./Cv3DPoseEstimate.d \
./Cv3DPoseEstimateDisp.d \
./Cv3DPoseEstimateDispSpaceRef.d \
./Cv3DPoseEstimateRef.d \
./Cv3DPoseEstimateSSE.d \
./CvLevMarq3D.d \
./CvLevMarqDispSpace.d \
./CvMat3X3.d \
./CvMatUtils.d \
./CvStereoCamModel.d \
./CvStereoCamParams.d \
./CvTest3DPoseEstimate.d \
./CvTestTimer.d \
./calib_stereo.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/opencv/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


