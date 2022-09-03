set(srcs "ads122c04_test_main.c"
    "../../../common/i2c_interface.c"
    "../../../ads122c04/ads122c04.c")

set(includes "../../../ads122c04")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${includes})
                       
