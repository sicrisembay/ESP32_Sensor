if(CONFIG_TC_DRIVER_FT6336)
    set(srcs "touch_test_main.c"
        "../../../common/i2c_interface.c"
        "../../../touchscreen/ft6x36/ft6x36.c")
elseif(CONFIG_TC_DRIVER_GT911)
    set(srcs "touch_test_main.c"
        "../../../common/i2c_interface.c"
        "../../../touchscreen/gt911/gt911.c")
endif()

if(CONFIG_TC_DRIVER_FT6336)
    set(includes "../../../common"
        "../../../touchscreen/include"
        "../../../touchscreen/ft6x36")
elseif(CONFIG_TC_DRIVER_GT911)
    set(includes "../../../common"
        "../../../touchscreen/include"
        "../../../touchscreen/gt911")
endif()

idf_component_register(SRCS ${srcs}
                        INCLUDE_DIRS ${includes})
                       
