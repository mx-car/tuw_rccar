add_subdirectory(example)

add_library(tuw_serial STATIC 
  ../../../../include/tuw/serial/serial_base.cpp
  serial_message.cpp )
SET_TARGET_PROPERTIES(tuw_serial PROPERTIES LINKER_LANGUAGE C)
