cmake_minimum_required(VERSION 2.8.3)
project(wsg50_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  dnb_msgs
  wsg50_common
  dnb_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs wsg50_common dnb_msgs
)

set(DRIVER_SOURCES 
  src/checksum.cpp include/wsg50/checksum.h
  src/cmd.c include/wsg50/cmd.h
  src/common.cpp include/wsg50/common.h
  src/functions.cpp include/wsg50/functions.h
  src/interface.cpp include/wsg50/interface.h
  src/main.cpp
  src/msg.c include/wsg50/msg.h
  src/serial.c include/wsg50/serial.h
  src/tcp.c include/wsg50/tcp.h
  src/udp.c include/wsg50/udp.h)

set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DOSNAME_LINUX)
add_definitions(-g)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(wsg50 src/main.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg50 ${catkin_LIBRARIES})
add_dependencies(wsg50 wsg50_common_generate_messages_cpp)

#############
## Install ##
#############
install(
  TARGETS wsg50
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY module_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/autotest.test)
#endif()