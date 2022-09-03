set(srcs "icm20600_test_main.c"
    "../../../common/i2c_interface.c"
    "../../../icm20600/icm20600.c")

set(includes "../../../icm20600")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${includes})
                       
