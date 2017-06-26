
add_executable(serial-arduino-shm main_arduino_shm.cpp serial_arduino_shm.cpp)
target_link_libraries(serial-arduino-shm tuw_serial_arduino shm_arduino_objects ${ShmFw_LIBRARIES} ${Boost_LIBRARIES})
