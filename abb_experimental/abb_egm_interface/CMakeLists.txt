cmake_minimum_required(VERSION 2.8.3)

project(abb_egm_interface)

find_package(catkin REQUIRED COMPONENTS control_msgs roscpp)

find_package(Boost REQUIRED COMPONENTS system thread)

##### Protobuf
find_package(Protobuf REQUIRED)

# Make sure protoc is present, as apparently the above find_package()
# doesn't check that.
find_program(Protobuf_PROTOC_LOC NAMES protoc)
if (NOT Protobuf_PROTOC_LOC)
  message(FATAL_ERROR "Cannot find required 'protoc', cannot process Protobuf files without it. Aborting.")
endif()

# Generate c++ for protocol classes (headers and sources get written to the
# CMAKE_CURRENT_BINARY_DIR location).
set(EgmProtoFiles proto/egm.proto proto/egm_interface.proto)
if (NOT QUIET)
  message(STATUS "Generating protobuf C++ for: ${EgmProtoFiles}")
endif()
protobuf_generate_cpp(EgmProtoSources EgmProtoHeaders ${EgmProtoFiles})



###################################
## Catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS control_msgs roscpp
  DEPENDS Boost Protobuf
)



###########
## Build ##
########### 
set(SRC_FILES
    include/abb_egm_interface/egm_common.h
    include/abb_egm_interface/egm_common_auxiliary.h
    include/abb_egm_interface/egm_common_math.h
    include/abb_egm_interface/egm_interface_default.h
    include/abb_egm_interface/egm_message_manager.h
    include/abb_egm_interface/egm_server.h
    include/abb_egm_interface/egm_simple_interpolation_manager.h
    ${EgmProtoHeaders}

    src/egm_common.cpp
    src/egm_common_auxiliary.cpp
    src/egm_common_math.cpp
    src/egm_interface_default.cpp
    src/egm_message_manager.cpp
    src/egm_server.cpp
    src/egm_simple_interpolation_manager.cpp
    ${EgmProtoSources}
)

include_directories(
  include
  # contains protobuf generated sources
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})
   
  
  
#############
## Install ##
############# 
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
  
install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
