cmake_minimum_required(VERSION 2.8.3)
project(cmake_tivac)

include(tm4c123g.cmake)

include_directories(include)

set(TIVA_WARE_PATH "$ENV{HOME}/projects/embedded/tivac/tivaware")
include_directories(${TIVA_WARE_PATH})

add_executable(${CMAKE_PROJECT_NAME}.axf startup_gcc.c src/main.cpp src/pin.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}.axf 
  ${TIVA_WARE_PATH}/usblib/gcc/libusb.a
  ${TIVA_WARE_PATH}/driverlib/gcc/libdriver.a
)
