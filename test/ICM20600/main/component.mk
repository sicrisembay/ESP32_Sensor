COMPONENT_OBJS += icm20600_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../common/i2c_interface.o
COMPONENT_SRCDIRS += ../../../common
COMPONENT_OBJS += ../../../icm20600/icm20600.o
COMPONENT_SRCDIRS += ../../../icm20600