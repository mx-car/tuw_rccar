add_library(firmware_servos_demo STATIC main_servos_demo.cpp)
SET_TARGET_PROPERTIES(firmware_servos_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_servos_demo tuw_servos core )
