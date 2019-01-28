cmake_minimum_required(VERSION 2.8.3)

project(abb_rws_interface)

find_package(catkin REQUIRED)

find_package(CURL REQUIRED)



###################################
## Catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS CURL
)



###########
## Build ##
###########
set(SRC_FILES
    include/abb_rws_interface/rws_client.h
    include/abb_rws_interface/rws_curl_client.h
    include/abb_rws_interface/rws_interface.h
    include/abb_rws_interface/rws_interface_common.h
    include/abb_rws_interface/rws_interface_yumi.h
    include/abb_rws_interface/rws_interface_yumi_common.h
    include/abb_rws_interface/rws_rapid_data.h

    src/rws_client.cpp
    src/rws_curl_client.cpp
    src/rws_interface.cpp
    src/rws_interface_yumi.cpp
    src/rws_rapid_data.cpp
)

include_directories(
  include
  ${CURL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SRC_FILES}) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CURL_LIBRARIES})



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
