#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := test_ADS122C04

# Get Path of ESP32_Sensor
ESP32_SENSOR_PATH := $(dir $(CURDIR))
ESP32_SENSOR_PATH := $(dir $(ESP32_SENSOR_PATH:%/=%))
$(info ESP32_SENSOR_PATH is $(ESP32_SENSOR_PATH))
export ESP32_SENSOR_PATH

ifndef IDF_PATH
$(error IDF_PATH variable is not set to a valid directory.)
endif

include $(IDF_PATH)/make/project.mk
