cmake_minimum_required(VERSION 3.13)

SET(APP usb_to_tty)

SET(CMAKE_C_LINK_FLAGS "")
SET(CMAKE_CXX_LINK_FLAGS "")
SET(CMAKE_ASM_LINK_FLAGS "")

SET(CMAKE_C_FLAGS "-std=gnu11")
SET(CMAKE_CXX_FLAGS "-std=gnu++11 -fno-rtti -fno-exceptions")
SET(CMAKE_ASM_FLAGS "")

project(${APP} C CXX ASM)

message("CMAKE_SYSTEM_NAME ${CMAKE_SYSTEM_NAME}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(MACOS_LIBS "-isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk -Wl,-framework,IOKit -Wl,-framework,CoreFoundation -Wl,-framework,Security")
  SET(PLATFORM_LINKER_FLAGS "-Wl,-dead_strip -Wl,-map,<TARGET>.map ${MACOS_LIBS}")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  SET(PLATFORM_LINKER_FLAGS "-Wl,-s -lpthread -lutil -ludev -Wl,--gc-sections -Wl,--as-needed -s -Wl,-Map=<TARGET>.map")
endif()

SET(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <OBJECTS> -o <TARGET> -lstdc++ <LINK_LIBRARIES> ${PLATFORM_LINKER_FLAGS}")
SET(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE}")

add_compile_options(-Os -g0 -ffunction-sections -ffreestanding)

set(DEPS_SRC_DIR $ENV{DEPS_SRC_DIR})
set(DEPS_BUILD_PATH $ENV{DEPS_BUILD_PATH})

function(SETVAR NAME VALUE)
  SET("${NAME}" "${VALUE}" CACHE STRING "" FORCE)
endfunction()

function(register_lib dep inc lib)
  string(TOUPPER ${dep} DEP)

  if(NOT DEFINED ARGV3)
    SET(dir "${dep}")
  else()
    SET(dir "${ARGV3}/${dep}")
  endif()

  SETVAR("${DEP}_DIR" "${DEPS_SRC_DIR}/${dir}")
  SETVAR("${DEP}_INCLUDES" "${DEPS_SRC_DIR}/${dir}/${inc}")
  SETVAR("${DEP}_STATIC_LIB" "${DEPS_BUILD_PATH}/${dir}/${lib}")
endfunction()

register_lib("libusb" "libusb" "libusb/.libs/libusb-1.0.a")

file(GLOB_RECURSE app_src CONFIGURE_DEPENDS "src/*.c" "src/*.cpp")

add_executable(${APP} ${app_src})
target_include_directories(${APP} PRIVATE ${LIBUSB_INCLUDES})
target_link_libraries(${APP} PRIVATE ${LIBUSB_STATIC_LIB})
