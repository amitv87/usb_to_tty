cmake_minimum_required(VERSION 3.13)

SET(APP usb_to_tty)

SET(CMAKE_C_LINK_FLAGS "")
SET(CMAKE_CXX_LINK_FLAGS "")
SET(CMAKE_ASM_LINK_FLAGS "")

SET(CMAKE_C_FLAGS "-std=gnu11")
SET(CMAKE_CXX_FLAGS "-std=gnu++11 -fno-rtti -fno-exceptions")
SET(CMAKE_ASM_FLAGS "")

SET(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <OBJECTS> -o <TARGET> -Wl,-Map=<TARGET>.map <LINK_LIBRARIES>")
SET(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE}")

project(${APP} C CXX ASM)

SET(LINKER_FLAGS "-lstdc++ -lutil -lusb-1.0 -Wl,-s -Wl,--gc-sections")
add_compile_options(-Os -g0 -fstrict-aliasing -ffunction-sections -ffreestanding -fexpensive-optimizations -frename-registers -fomit-frame-pointer)

FILE(GLOB_RECURSE app_src CONFIGURE_DEPENDS "src/*.c" "src/*.cpp")

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES "include" "libusb" "libusb-1.0")

message("LIBUSB_INCLUDE_DIR ${LIBUSB_INCLUDE_DIR}")

ADD_EXECUTABLE(${APP} ${app_src})
target_include_directories(${APP} PRIVATE ${LIBUSB_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${APP} PRIVATE ${LINKER_FLAGS})
