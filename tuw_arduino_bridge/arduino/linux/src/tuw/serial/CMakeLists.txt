
add_library(tuw_serial_arduino
   TimeoutSerial.cpp
   ../../../../include/tuw/serial/serial_base.cpp
   serial_message.cpp
   serial_arduino.cpp
)
target_link_libraries(tuw_serial_arduino ${Boost_LIBRARIES})
add_subdirectory(example)
