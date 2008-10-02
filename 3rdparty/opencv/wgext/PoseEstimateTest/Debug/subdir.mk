################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CvTest3DPoseEstimate.cpp 

OBJS += \
./CvTest3DPoseEstimate.o 

CPP_DEPS += \
./CvTest3DPoseEstimate.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/opencv/include -I"/wg/stor1/jdchen/workspace/ros-pkg/3rdparty/opencv/wgext/3DPoseEstimation/include" -I/u/jdchen/workspace/ros-pkg/3rdparty/VTK/vtk-install/include/vtk-5.0/ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


