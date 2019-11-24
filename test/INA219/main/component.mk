COMPONENT_OBJS += ina219_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../common/i2c_interface.o
COMPONENT_SRCDIRS += ../../../common
COMPONENT_OBJS += ../../../ina219/ina219.o
COMPONENT_SRCDIRS += ../../../ina219