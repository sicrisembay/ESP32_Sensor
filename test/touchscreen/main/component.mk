COMPONENT_OBJS += touch_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../common/i2c_interface.o
COMPONENT_SRCDIRS += ../../../common

#ifdef CONFIG_TC_DRIVER_FT6336
COMPONENT_OBJS += ../../../touchscreen/ft6x36/ft6x36.o
COMPONENT_SRCDIRS += ../../../touchscreen/ft6x36
#endif