add_library(firmware_sandbox_demo STATIC main_sandbox_demo.cpp)
SET_TARGET_PROPERTIES(firmware_sandbox_demo PROPERTIES LINKER_LANGUAGE C)  

arduino_upload(firmware_sandbox_demo core )
