set(srcs "sht21_test_main.c"
    "../../../common/i2c_interface.c"
    "../../../sht21/sht21.c")

set(includes "../../../sht21")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${includes})
                       
