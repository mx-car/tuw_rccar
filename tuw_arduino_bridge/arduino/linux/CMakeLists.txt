cmake_minimum_required(VERSION 2.6)
project(arduino_linux)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++11) ## for c++ 2011
find_package (Boost 1.40.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals system chrono)

if (DEFINED ENV{SHMFW})
  include_directories($ENV{SHMFW}/include/)
  include_directories($ENV{SHMFW}/src/)
  set(ShmFw_LIBRARIES
   $ENV{SHMFW}/lib/libshmfw_core.so
   $ENV{SHMFW}/lib/libshmfw_objects.so
  )
else()
  find_package(ShmFw 2014.0.0 REQUIRED)
endif()

find_package(Curses)
IF(Curses_FOUND)
  MESSAGE( STATUS ${CURSES_LIBRARY})
  SET(Curses_FOUND 1)
ELSE ()
  MESSAGE( STATUS "Trying : pkg_check_modules ( ncurses++ REQUIRED ncurses++  )" )
  find_package (PkgConfig REQUIRED)
  pkg_check_modules ( ncurses++ REQUIRED ncurses++  )
  MESSAGE( STATUS "CURSES_LIBRARY:     ${CURSES_LIBRARY}" )
  IF(CURSES_LIBRARY)
    SET(Curses_FOUND 1)
  else()
    MESSAGE( WARNING "--> NCURSES: Ubuntu 14.04: \"sudo apt-get install libncurses5-dev\" ")
  endif()
ENDIF()

include_directories(include)
add_subdirectory(src)