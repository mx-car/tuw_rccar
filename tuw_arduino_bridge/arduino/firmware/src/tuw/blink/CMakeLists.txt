add_library(firmware_blink_demo STATIC main_blink_demo.cpp)
SET_TARGET_PROPERTIES(firmware_blink_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_blink_demo core )
