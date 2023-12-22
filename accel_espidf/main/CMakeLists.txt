# Edit following two lines to set component requirements (see docs)
# set(COMPONENT_REQUIRES )
# set(COMPONENT_PRIV_REQUIRES )

# set(COMPONENT_SRCS 
#     "main.c"
#    
#     "motor.h"
#     "accel.c"
#     "accel.h")

idf_component_register(SRCS 
    "main.c"
    "motor.c"
    "motor.h"
    "accel.c"
    "accel.h"
   
    "encoder.h"
    INCLUDE_DIRS "."
    REQUIRES ahrs mpu9250 rotary_encoder esp_timer)

set(COMPONENT_ADD_INCLUDEDIRS "")

# register_component()
