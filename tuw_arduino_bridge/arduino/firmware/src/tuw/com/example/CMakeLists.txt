add_library(firmware_com_demo STATIC main_com_demo.cpp)
SET_TARGET_PROPERTIES(firmware_com_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_com_demo tuw_serial core )
