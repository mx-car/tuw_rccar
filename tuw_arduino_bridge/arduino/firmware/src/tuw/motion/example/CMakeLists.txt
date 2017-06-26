add_library(firmware_motion_demo STATIC main_motion_demo.cpp)
SET_TARGET_PROPERTIES(firmware_motion_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_motion_demo tuw_motion tuw_serial tuw_servos tuw_bldc tuw_imu core )
