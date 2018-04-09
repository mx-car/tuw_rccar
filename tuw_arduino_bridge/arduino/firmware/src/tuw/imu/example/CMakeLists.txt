add_library(firmware_imu_demo STATIC main_imu_demo.cpp)
SET_TARGET_PROPERTIES(firmware_imu_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_imu_demo tuw_imu core )
