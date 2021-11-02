# Install script for directory: /home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yoga/my_work/catkin_yoga/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master/catkin_generated/installspace/livox_relocalization.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/livox_relocalization/cmake" TYPE FILE FILES
    "/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master/catkin_generated/installspace/livox_relocalizationConfig.cmake"
    "/home/yoga/my_work/catkin_yoga/build/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master/catkin_generated/installspace/livox_relocalizationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/livox_relocalization" TYPE FILE FILES "/home/yoga/my_work/catkin_yoga/src/TX2_StereoSLAM-master/src/loam_only/loam_only_relocal/src/livox_relocalization-master/package.xml")
endif()

