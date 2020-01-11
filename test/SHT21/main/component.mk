COMPONENT_OBJS += sht21_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../common/i2c_interface.o
COMPONENT_SRCDIRS += ../../../common
COMPONENT_OBJS += ../../../sht21/sht21.o
COMPONENT_SRCDIRS += ../../../sht21