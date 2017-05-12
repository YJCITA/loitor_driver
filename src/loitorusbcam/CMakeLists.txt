cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 OLD) # cmake 3.1以后版本新特性
#project(loitorusbcam)
# set(CMAKE_C_COMPILER g++)

link_directories("../loitorimu")

add_library(loitorusbcam STATIC loitorusbcam.cpp)
target_link_libraries(loitorusbcam loitorimu)
