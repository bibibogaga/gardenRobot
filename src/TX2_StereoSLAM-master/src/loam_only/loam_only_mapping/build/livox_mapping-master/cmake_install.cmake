# Install script for directory: /home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/src/livox_mapping-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/build/livox_mapping-master/catkin_generated/installspace/livox_mapping.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/livox_mapping/cmake" TYPE FILE FILES
    "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/build/livox_mapping-master/catkin_generated/installspace/livox_mappingConfig.cmake"
    "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/build/livox_mapping-master/catkin_generated/installspace/livox_mappingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/livox_mapping" TYPE FILE FILES "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_mapping/src/livox_mapping-master/package.xml")
endif()

