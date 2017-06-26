add_library(firmware_bldc_demo STATIC main_bldc_demo.cpp)
SET_TARGET_PROPERTIES(firmware_bldc_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_bldc_demo tuw_bldc core )
