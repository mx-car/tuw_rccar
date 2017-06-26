add_subdirectory(example)

add_library(tuw_servos STATIC servos.cpp )
SET_TARGET_PROPERTIES(tuw_servos PROPERTIES LINKER_LANGUAGE C)
