cmake_minimum_required(VERSION 2.8)
project( pan_tilt )

if (POLICY CMP0015)
  cmake_policy(SET CMP0015 NEW)
endif (POLICY CMP0015)

find_package( Boost 1.47 REQUIRED COMPONENTS system-mt filesystem-mt thread-mt )
find_package( Protobuf REQUIRED )

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

set( BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../coral )
set( INCLUDE_ROOT "${BASE_DIR}/includes" )
set( CORE_INCLUDE ${INCLUDE_ROOT}/core )
set( NETAPP_INCLUDE ${INCLUDE_ROOT}/netapp )
include_directories( "${CORE_INCLUDE}" )
include_directories( "${CORE_INCLUDE}/crypto" )
include_directories( "${CORE_INCLUDE}/file_system" )
include_directories( "${CORE_INCLUDE}/cli" )
include_directories( "${CORE_INCLUDE}/helpers" )
include_directories( "${CORE_INCLUDE}/container" )
include_directories( "${CORE_INCLUDE}/concurrency" )
include_directories( "${CORE_INCLUDE}/time" )
include_directories( "${CORE_INCLUDE}/logging" )
include_directories( "${NETAPP_INCLUDE}/common" )
include_directories( "${NETAPP_INCLUDE}/server" )
include_directories( "${INCLUDE_ROOT}/rpc" )
include_directories( ../common/include )
include_directories( ../common/rpc_include )
include_directories( rpc_include )

link_directories( ${BASE_DIR}/lib/core )
link_directories( ${BASE_DIR}/lib/netapp )
link_directories( ${BASE_DIR}/lib/rpc )

add_library( scanner_common STATIC 
   i2c_interface.cpp
   pwm_controller.cpp
   ads1115_interface.cpp
   pan_tilt_controller.cpp
   pan_tilt_thread.cpp
   lidar_lite.cpp )

add_executable(
   pan_tilt_util
   pan_tilt_util.cpp )

add_executable(
   scanner_service
   ../common/rpc_source/ScannerRpc.pb.cc
   rpc_source/ScannerServiceServerStub.cpp
   rpc_source/ScannerServiceSetModeAction.cpp
   scanner_service.cpp )

add_executable(
   serial_scanner_service
   serial_scanner_service.cpp )

target_link_libraries( pan_tilt_util scanner_common )
target_link_libraries( pan_tilt_util coral_core_shared )
target_link_libraries( pan_tilt_util ${Boost_LIBRARIES})
target_link_libraries( pan_tilt_util ${PROTOBUF_LIBRARIES} )
target_link_libraries( pan_tilt_util pthread )

target_link_libraries( scanner_service scanner_common )
target_link_libraries( scanner_service coral_core_shared )
target_link_libraries( scanner_service coral_netapp_common_shared )
target_link_libraries( scanner_service coral_netapp_server_shared )
target_link_libraries( scanner_service coral_rpc_shared )
target_link_libraries( scanner_service ${Boost_LIBRARIES})
target_link_libraries( scanner_service ${PROTOBUF_LIBRARIES} )
target_link_libraries( scanner_service pthread )

target_link_libraries( serial_scanner_service scanner_common )
target_link_libraries( serial_scanner_service coral_core_shared )
target_link_libraries( serial_scanner_service coral_netapp_common_shared )
target_link_libraries( serial_scanner_service coral_netapp_server_shared )
target_link_libraries( serial_scanner_service coral_rpc_shared )
target_link_libraries( serial_scanner_service ${Boost_LIBRARIES})
target_link_libraries( serial_scanner_service ${PROTOBUF_LIBRARIES} )
target_link_libraries( serial_scanner_service pthread )
