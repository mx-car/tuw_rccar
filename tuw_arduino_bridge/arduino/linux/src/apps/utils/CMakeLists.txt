IF(Curses_FOUND)
add_executable(arduino-object-editor main_arduino_object_editor.cpp )
target_link_libraries(arduino-object-editor shm_arduino_objects ${ShmFw_LIBRARIES} ${CURSES_LIBRARY} ${Boost_LIBRARIES})

add_executable(arduino-object-viewer main_arduino_object_viewer.cpp )
target_link_libraries(arduino-object-viewer shm_arduino_objects ${ShmFw_LIBRARIES} ${CURSES_LIBRARY} ${Boost_LIBRARIES})
ENDIF()

add_executable(arduino-teleop main_arduino_teleop.cpp )
target_link_libraries(arduino-teleop shm_arduino_objects ${ShmFw_LIBRARIES}  ${Boost_LIBRARIES})


add_executable(arduino-object-change main_arduino_object_change.cpp )
target_link_libraries(arduino-object-change shm_arduino_objects ${ShmFw_LIBRARIES} ${Boost_LIBRARIES})