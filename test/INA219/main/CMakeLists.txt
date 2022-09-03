set(srcs "ina219_test_main.c"
    "../../../common/i2c_interface.c"
    "../../../ina219/ina219.c")

set(includes "../../../ina219")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${includes})
                       
