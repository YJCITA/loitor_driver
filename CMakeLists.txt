cmake_minimum_required(VERSION 2.8)
project(camtest2)

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

######################### configuration ###########################
OPTION(BUILD_RPi_PROGRAM "build RPi program" OFF)
IF (BUILD_RPi_PROGRAM)
    MESSAGE(STATUS "build RPi program")
ELSE()
    MESSAGE(STATUS "build linux program")
ENDIF()

SET(BUILD_RPi 0) # 0:编译UUBUNTU版本　　１：编译RPi版本
SET(DEBUG_UBUNTU 1) # 0:release版本　　１：UBUNTU端debug版本
SET(CMAKE_BUILD_TYPE "Debug")

MESSAGE(STATUS "PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
IF(BUILD_RPi_PROGRAM)
    # 指定opencv 3.1本地库
    set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencv/opencv_3.1_rpi_pc/share/OpenCV")
    FIND_PACKAGE(OpenCV REQUIRED)
    MESSAGE(STATUS "OpenCV_DIR=${OpenCV_DIR}")
    SET(CMAKE_SYSTEM_NAME RPi)#告知当前使用的是交叉编译方式，必须配置  实际调用的是本地：/usr／bin中的编译器
    MESSAGE(STATUS "TOOLCHAIN_DIR=${TOOLCHAIN_DIR}")
    SET(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR})
    SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    SET(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")
    SET(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
    MESSAGE(STATUS "CMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
ELSE()
    SET(OpenCV_DIR "/home/yj/bak/packet/opencv/opencv3.1/install/share/OpenCV")
#     SET(OpenCV_DIR "/home/yj/bak/packet/opencv/opencv-2.4.13/install/share/OpenCV")
    FIND_PACKAGE(OpenCV REQUIRED)
    MESSAGE(OpenCV_DIR "OpenCV_DIR=${OpenCV_DIR}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
    
ENDIF()


IF(BUILD_RPi_PROGRAM)
    include_directories("${PROJECT_SOURCE_DIR}/thirdparty/libusb/include/")
    link_directories("${PROJECT_SOURCE_DIR}/thirdparty/libusb/lib/rpi")
ENDIF()
    
include_directories("${PROJECT_SOURCE_DIR}/src/loitorusbcam")
include_directories("${PROJECT_SOURCE_DIR}/src/loitorimu")

link_directories("${PROJECT_BINARY_DIR}/src/loitorusbcam")
link_directories("${PROJECT_BINARY_DIR}/src/loitorimu")

add_subdirectory(src/)
add_subdirectory(src/loitorusbcam)
add_subdirectory(src/loitorimu)
