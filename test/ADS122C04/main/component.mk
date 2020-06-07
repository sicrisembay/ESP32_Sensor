COMPONENT_OBJS += ads122c04_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../common/i2c_interface.o
COMPONENT_SRCDIRS += ../../../common
COMPONENT_OBJS += ../../../ads122c04/ads122c04.o
COMPONENT_SRCDIRS += ../../../ads122c04