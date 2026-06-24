cmake_minimum_required(VERSION 3.20)
project(kinectcam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)  # -> -O2
endif()

include(GNUInstallDirs)

# --- Locate libfreenect2: prefer its CMake config package, fall back to pkg-config ---
find_package(freenect2 QUIET)
if(freenect2_FOUND)
  message(STATUS "libfreenect2: find_package (${freenect2_INCLUDE_DIRS})")
  set(F2_INCLUDES ${freenect2_INCLUDE_DIRS})
  set(F2_LIBS     ${freenect2_LIBRARIES})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(F2 REQUIRED freenect2)
  message(STATUS "libfreenect2: pkg-config (${F2_INCLUDE_DIRS})")
  set(F2_INCLUDES ${F2_INCLUDE_DIRS})
  set(F2_LIBS     ${F2_LIBRARIES})
  link_directories(${F2_LIBRARY_DIRS})
endif()

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(kinect_bridge main.cpp)
target_include_directories(kinect_bridge PRIVATE ${F2_INCLUDES} ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(kinect_bridge PRIVATE ${F2_LIBS} ${LIBUSB_LIBRARIES} Threads::Threads)

install(TARGETS kinect_bridge RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# --- Optional: drop the systemd unit + v4l2loopback module configs into /etc ---
#     cmake -B build -DINSTALL_SYSTEM_FILES=ON   (then sudo cmake --install build)
option(INSTALL_SYSTEM_FILES "Install systemd unit and v4l2loopback configs to /etc" OFF)
if(INSTALL_SYSTEM_FILES)
  set(BRIDGE_BIN "${CMAKE_INSTALL_FULL_BINDIR}/kinect_bridge")

  file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/kinect-bridge.service" @ONLY CONTENT
[==[[Unit]
Description=Kinect v2 -> v4l2loopback bridge (IR unlock + RGB webcam)
After=multi-user.target

[Service]
ExecStart=@BRIDGE_BIN@
Environment=KINECT_IR_DEV=/dev/video10
Environment=KINECT_RGB_DEV=/dev/video11
Environment=KINECT_IR_AUTO=1
Restart=no
RuntimeMaxSec=30

[Install]
WantedBy=multi-user.target
]==])

  file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/modprobe-v4l2loopback.conf" CONTENT
[==[options v4l2loopback devices=2 video_nr=10,11 card_label="KinectIR","KinectRGB" exclusive_caps=1,1
]==])

  file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/load-v4l2loopback.conf" CONTENT "v4l2loopback\n")

  file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/65-kinect2.rules" CONTENT
[==[# Kinect v2 — grant seat access and make power/control writable for autosuspend
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", TAG+="uaccess", RUN+="/bin/chmod 0666 /sys%p/power/control /sys%p/power/autosuspend_delay_ms"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", TAG+="uaccess"
]==])

  install(FILES "${CMAKE_BINARY_DIR}/kinect-bridge.service"        DESTINATION /etc/systemd/system)
  install(FILES "${CMAKE_BINARY_DIR}/modprobe-v4l2loopback.conf"   DESTINATION /etc/modprobe.d    RENAME v4l2loopback.conf)
  install(FILES "${CMAKE_BINARY_DIR}/load-v4l2loopback.conf"       DESTINATION /etc/modules-load.d RENAME v4l2loopback.conf)
  install(FILES "${CMAKE_BINARY_DIR}/65-kinect2.rules"             DESTINATION /etc/udev/rules.d)
endif()