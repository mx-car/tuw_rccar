add_subdirectory(example)
add_library(tuw_imu STATIC imu.cpp i2c.cpp Wire.cpp)
SET_TARGET_PROPERTIES(tuw_imu PROPERTIES LINKER_LANGUAGE C)
