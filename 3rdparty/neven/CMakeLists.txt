cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(neven)

set(ROS_BUILD_TYPE Release)

include_directories(${PROJECT_SOURCE_DIR}/nevensrc/FaceRecEm/common/src/b_FDSDK
        ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/conf/b_BasicEm
        ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/src
        ${PROJECT_SOURCE_DIR}/include/neven
        ${PROJECT_SOURCE_DIR}/data)

ADD_CUSTOM_COMMAND(OUTPUT data/RFFspeed_501.i
                   COMMAND ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFspeed_501.bmd > ${PROJECT_SOURCE_DIR}/data/RFFspeed_501.i
                   DEPENDS ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFspeed_501.bmd)

ADD_CUSTOM_COMMAND(OUTPUT data/RFFstd_501.i
                   COMMAND ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFstd_501.bmd > ${PROJECT_SOURCE_DIR}/data/RFFstd_501.i
                   DEPENDS ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFstd_501.bmd)

ADD_CUSTOM_COMMAND(OUTPUT data/RFFprec_501.i
                   COMMAND ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFprec_501.bmd > ${PROJECT_SOURCE_DIR}/data/RFFprec_501.i
                   DEPENDS ${PROJECT_SOURCE_DIR}/scripts/bin2c.py  ${PROJECT_SOURCE_DIR}/nevensrc/Embedded/common/data/APIEm/Modules/RFFprec_501.bmd)

ADD_CUSTOM_TARGET(generate_data_includes ALL DEPENDS data/RFFspeed_501.i data/RFFstd_501.i data/RFFprec_501.i)

rospack_add_library(neven 
       src/neven.cpp
       nevensrc/Embedded/common/src/b_APIEm/DCR.c 
       nevensrc/Embedded/common/src/b_APIEm/BFFaceFinder.c 
       nevensrc/Embedded/common/src/b_APIEm/FaceFinder.c 
       nevensrc/Embedded/common/src/b_APIEm/FaceFinderRef.c 
       nevensrc/Embedded/common/src/b_APIEm/Functions.c 
       nevensrc/Embedded/common/src/b_BasicEm/APh.c 
       nevensrc/Embedded/common/src/b_BasicEm/APhArr.c 
       nevensrc/Embedded/common/src/b_BasicEm/Complex.c 
       nevensrc/Embedded/common/src/b_BasicEm/ComplexArr.c 
       nevensrc/Embedded/common/src/b_BasicEm/Context.c 
       nevensrc/Embedded/common/src/b_BasicEm/DynMemManager.c 
       nevensrc/Embedded/common/src/b_BasicEm/Functions.c 
       nevensrc/Embedded/common/src/b_BasicEm/Int16Arr.c 
       nevensrc/Embedded/common/src/b_BasicEm/Int32Arr.c 
       nevensrc/Embedded/common/src/b_BasicEm/Int8Arr.c 
       nevensrc/Embedded/common/src/b_BasicEm/MathSSE2.c
       nevensrc/Embedded/common/src/b_BasicEm/Math.c
       nevensrc/Embedded/common/src/b_BasicEm/MemSeg.c 
       nevensrc/Embedded/common/src/b_BasicEm/MemTbl.c 
       nevensrc/Embedded/common/src/b_BasicEm/Memory.c 
       nevensrc/Embedded/common/src/b_BasicEm/Phase.c 
       nevensrc/Embedded/common/src/b_BasicEm/String.c 
       nevensrc/Embedded/common/src/b_BasicEm/UInt16Arr.c 
       nevensrc/Embedded/common/src/b_BasicEm/UInt32Arr.c 
       nevensrc/Embedded/common/src/b_BasicEm/UInt8Arr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/BitParam.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/Feature.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/Functions.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/I04Dns2x2Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/I04Dns2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/I04Tld2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L01Dns2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L01Tld1x1Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L01Tld2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L04Dns2x2Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L04Dns2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L04Dns3x3Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L04Tld2x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L06Dns3x3Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L06Dns4x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/L06DnsNx4x4Ftr.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/LocalScanDetector.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/LocalScanner.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/ScanDetector.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/Scanner.c 
       nevensrc/Embedded/common/src/b_BitFeatureEm/Sequence.c 
       nevensrc/Embedded/common/src/b_ImageEm/APhImage.c 
       nevensrc/Embedded/common/src/b_ImageEm/ComplexImage.c 
       nevensrc/Embedded/common/src/b_ImageEm/Flt16Image.c 
       nevensrc/Embedded/common/src/b_ImageEm/Functions.c 
       nevensrc/Embedded/common/src/b_ImageEm/HistoEq.c 
       nevensrc/Embedded/common/src/b_ImageEm/UInt16ByteImage.c 
       nevensrc/Embedded/common/src/b_ImageEm/UInt16BytePyrImage.c 
       nevensrc/Embedded/common/src/b_ImageEm/UInt8Image.c 
       nevensrc/Embedded/common/src/b_ImageEm/UInt32Image.c 
       nevensrc/Embedded/common/src/b_ImageEm/UInt8PyramidalImage.c 
       nevensrc/Embedded/common/src/b_TensorEm/Alt.c 
       nevensrc/Embedded/common/src/b_TensorEm/Cluster2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Cluster3D.c 
       nevensrc/Embedded/common/src/b_TensorEm/CompactAlt.c 
       nevensrc/Embedded/common/src/b_TensorEm/CompactMat.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Alt2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Alt3D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Mat2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Mat3D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Vec.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Vec2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Flt16Vec3D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Functions.c 
       nevensrc/Embedded/common/src/b_TensorEm/IdCluster2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Int16Mat2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Int16Rect.c 
       nevensrc/Embedded/common/src/b_TensorEm/Int16Vec2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Int16Vec3D.c 
       nevensrc/Embedded/common/src/b_TensorEm/Int32Mat.c 
       nevensrc/Embedded/common/src/b_TensorEm/MapSequence.c 
       nevensrc/Embedded/common/src/b_TensorEm/Mat.c 
       nevensrc/Embedded/common/src/b_TensorEm/Normalizer.c 
       nevensrc/Embedded/common/src/b_TensorEm/RBFMap2D.c 
       nevensrc/Embedded/common/src/b_TensorEm/SubVecMap.c 
       nevensrc/Embedded/common/src/b_TensorEm/Uint32Rect.c 
       nevensrc/Embedded/common/src/b_TensorEm/VectorMap.c 
       nevensrc/FaceRecEm/common/src/b_FDSDK/DCR.c 
       nevensrc/FaceRecEm/common/src/b_FDSDK/FaceFinder.c
       nevensrc/FaceRecEm/common/src/b_FDSDK/SDK.c)
