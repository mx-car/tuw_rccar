add_subdirectory(example)

add_library(tuw_bldc STATIC pid.cpp bldc.cpp)
SET_TARGET_PROPERTIES(tuw_bldc PROPERTIES LINKER_LANGUAGE C)

