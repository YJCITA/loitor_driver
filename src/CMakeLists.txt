cmake_minimum_required(VERSION 2.8)

add_executable(camtest2 camtest2.cpp)
target_link_libraries(camtest2 loitorusbcam loitorimu libusb-1.0.so.0 ${OpenCV_LIBS}) #usb-1.0
