# This file is based on the work of:
#
# http://mjo.tc/atelier/2009/02/arduino-cli.html
# http://johanneshoff.com/arduino-command-line.html
# http://www.arduino.cc/playground/Code/CmakeBuild
# http://www.tmpsantos.com.br/en/2010/12/arduino-uno-ubuntu-cmake/
# http://forum.arduino.cc/index.php?topic=244741.0


set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/lib/m0plus")
set(LIBRARY_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/lib/m0plus")
set(TUNNING_FLAGS "") 
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_BOARD_FLAGS "-DF_CPU=${ARDUINO_FCPU} -DARDUINO_=10707 -DARDUINO_SAMD_ZERO -DARDUINO_ARCH_SAMD -D__SAMD21G18A__ -DUSB_VID=0x2a03 -DUSB_PID=0x804f -DUSBCON -DUSB_MANUFACTURER= -DUSB_PRODUCT=\"\\\"Arduino M0 Pro\\\"\"")
set(CMAKE_C_FLAGS   "-g -Os -w -std=gnu11   -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -mcpu=${ARDUINO_MCU} -mthumb ${CMAKE_BOARD_FLAGS}")
set(CMAKE_CXX_FLAGS "-g -Os -w -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -mcpu=${ARDUINO_MCU} -mthumb ${CMAKE_BOARD_FLAGS}  -fno-threadsafe-statics  -fno-rtti -fno-exceptions")
set(CMAKE_ASM_FLAGS "-g -x assembler-with-cpp                                                                                                                   ${CMAKE_BOARD_FLAGS}")


set(ARDUINO_AVRDUDE_CONFIG "${ARDUINO_ROOT}/hardware/tools/avr/etc/avrdude.conf")
set(ARDUINO_CORE_DIR "${ARDUINO_ROOT}/hardware/arduino/samd/cores/arduino")
set(ARDUINO_CMSIS_DIR "${ARDUINO_ROOT}/hardware/tools/CMSIS/CMSIS/Include/")
set(ARDUINO_DEVICE_DIR "${ARDUINO_ROOT}/hardware/tools/CMSIS/Device/ATMEL/")
set(ARDUINO_SCRIPTS "${ARDUINO_ROOT}/hardware/tools/OpenOCD-0.9.0-arduino/share/openocd/scripts/")
set(ARDUINO_BOARD_DIR "${ARDUINO_ROOT}/hardware/arduino/samd/variants/${ARDUINO_BOARD}")
set(ARDUINO_OPENOCD_CFG "${ARDUINO_BOARD_DIR}/openocd_scripts/arduino_zero.cfg")

#include_directories("/usr/include/c++/4.8")
include_directories(${ARDUINO_CMSIS_DIR})
include_directories(${ARDUINO_DEVICE_DIR})
include_directories(${ARDUINO_CORE_DIR})
include_directories(${ARDUINO_BOARD_DIR})

set(ARDUINO_SOURCE_FILES
        ${ARDUINO_BOARD_DIR}/variant.cpp
	${ARDUINO_CORE_DIR}/wiring_digital.c
	${ARDUINO_CORE_DIR}/USB/samd21_device.c
	${ARDUINO_CORE_DIR}/USB/samd21_host.c
	${ARDUINO_CORE_DIR}/wiring.c
	${ARDUINO_CORE_DIR}/delay.c
	${ARDUINO_CORE_DIR}/startup.c
	${ARDUINO_CORE_DIR}/WInterrupts.c
	${ARDUINO_CORE_DIR}/syscalls.c
	${ARDUINO_CORE_DIR}/avr/dtostrf.c
	${ARDUINO_CORE_DIR}/wiring_shift.c
	${ARDUINO_CORE_DIR}/hooks.c
	${ARDUINO_CORE_DIR}/itoa.c
	${ARDUINO_CORE_DIR}/wiring_analog.c
	
	${ARDUINO_CORE_DIR}/Uart.cpp
	${ARDUINO_CORE_DIR}/WMath.cpp
	${ARDUINO_CORE_DIR}/USB/HID.cpp
	${ARDUINO_CORE_DIR}/USB/USBCore.cpp
        ${ARDUINO_CORE_DIR}/USB/CDC.cpp
        ${ARDUINO_CORE_DIR}/IPAddress.cpp
        ${ARDUINO_CORE_DIR}/wiring_pulse.cpp
        ${ARDUINO_CORE_DIR}/Tone.cpp
        ${ARDUINO_CORE_DIR}/Print.cpp
        ${ARDUINO_CORE_DIR}/WString.cpp
        ${ARDUINO_CORE_DIR}/Stream.cpp
        ${ARDUINO_CORE_DIR}/Reset.cpp
        ${ARDUINO_CORE_DIR}/RingBuffer.cpp
        ${ARDUINO_CORE_DIR}/SERCOM.cpp
        ${ARDUINO_CORE_DIR}/main.cpp
        
        
)

add_library(core STATIC  ${ARDUINO_SOURCE_FILES})

# you have to call the function with quoates for the TRAGET_SOURCE_FILES
# e.g. arduino(example "${SOURCE_FILES}")
macro(arduino TRAGET_NAME TRAGET_SOURCE_FILES )

   
  add_library(${TRAGET_NAME} STATIC ${TRAGET_SOURCE_FILES} )
  SET_TARGET_PROPERTIES(${TRAGET_NAME} PROPERTIES LINKER_LANGUAGE C)  
    
  
  # uploads the bin 
  set(TARGET_UPLOAD ${TRAGET_NAME}.upload)
  add_custom_target(${TARGET_UPLOAD})
  add_dependencies(${TARGET_UPLOAD} ${TRAGET_NAME} core)
  add_custom_command(TARGET ${TARGET_UPLOAD} POST_BUILD
    COMMAND cd ${LIBRARY_OUTPUT_PATH} && ${ARDUINO_AR} crsT ${TARGET_UPLOAD}.a lib${TRAGET_NAME}.a libcore.a
    COMMAND ${CMAKE_C_COMPILER} -Os -Wl,--gc-sections -save-temps -T${ARDUINO_BOOTLOADER} --specs=nano.specs --specs=nosys.specs -mcpu=${ARDUINO_MCU} -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,-Map,${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.map -o ${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.elf -lm ${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.a
    COMMAND ${ARDUINO_OBJCOPY} -O binary ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.elf ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.bin
    COMMAND ${ARDUINO_OPENOCD} -d2 -s ${ARDUINO_SCRIPTS} -f ${ARDUINO_OPENOCD_CFG} -c "program ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.bin verify 0x4000 reset exit" 
  )      
endmacro()


# you have to call the function with quoates for the TRAGET_SOURCE_FILES
# e.g. arduino(example "${SOURCE_FILES}")
macro(arduino_upload TRAGET_NAME)   
  # uploads the bin 
  set(TARGET_UPLOAD ${TRAGET_NAME}.upload)
  add_custom_target(${TARGET_UPLOAD})
  add_dependencies(${TARGET_UPLOAD} ${TRAGET_NAME})
  set(STATIC_LIBARIES "lib${TRAGET_NAME}.a")
  foreach (arg ${ARGN})
    add_dependencies(${TARGET_UPLOAD} ${arg})
    set(STATIC_LIBARIES ${STATIC_LIBARIES} lib${arg}.a)
  endforeach ()
  
  file(REMOVE ${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.a)
  add_custom_command(TARGET ${TARGET_UPLOAD} POST_BUILD
    COMMAND cd ${LIBRARY_OUTPUT_PATH} && ${ARDUINO_AR} crsT ${TARGET_UPLOAD}.a ${STATIC_LIBARIES}
    COMMAND ${CMAKE_C_COMPILER} -Os -Wl,--gc-sections -save-temps -T${ARDUINO_BOOTLOADER} --specs=nano.specs --specs=nosys.specs -mcpu=${ARDUINO_MCU} -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,-Map,${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.map -o ${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.elf -lm ${LIBRARY_OUTPUT_PATH}/${TARGET_UPLOAD}.a -lm
    COMMAND ${ARDUINO_OBJCOPY} -O binary ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.elf ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.bin
    COMMAND ${ARDUINO_OPENOCD} -d2 -s ${ARDUINO_SCRIPTS} -f ${ARDUINO_OPENOCD_CFG} -c "program ${EXECUTABLE_OUTPUT_PATH}/${TARGET_UPLOAD}.bin verify 0x4000 reset exit" 
  )         
endmacro()


  
  
