# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
    "main.c"
    "motor.c"
    "motor.h"
    "accel.c"
    "accel.h")
set(COMPONENT_ADD_INCLUDEDIRS "")



idf_component_register(COMPONENT_SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ahrs mpu9250)

# register_component()
