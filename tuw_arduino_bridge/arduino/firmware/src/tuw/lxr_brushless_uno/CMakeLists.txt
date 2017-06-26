
set(SOURCE_FILES
    main_lxr_brushless_uno.cpp
    LXR_Brushless_Motorshield.cpp
)
include_directories(./)

arduino(lxrobotics ${SOURCE_FILES})