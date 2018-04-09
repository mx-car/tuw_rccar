add_subdirectory(example)


if(DEFINED $ENV{VEHICLE_TYPE})
    message(FATAL_ERROR "VEHICLE_TYPE environment variable defined add \"EXPORT VEHICLE_TYPE=1\" to your .bashrc")
endif()
add_definitions(-DVEHICLE_TYPE=$ENV{VEHICLE_TYPE})

add_library(tuw_motion STATIC motion.cpp)
SET_TARGET_PROPERTIES(tuw_motion  tuw_serial PROPERTIES LINKER_LANGUAGE C)
