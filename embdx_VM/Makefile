#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2015
# All rights reserved
# http://embedxcode.weebly.com
#
# Last update: Mar 20, 2015 release 270


# Libraries
#Application libraries
APP_LIBS_LIST = SPI

#User Libraries
USER_LIBS_LIST =  #Adafruit_BLE_UART



# Parameters

# SERIAL_BAUDRATE for the serial console, 9600 by default
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 115200


# Miscellaneous
# ----------------------------------
# Manage path with space in the name
#
CURRENT_DIR   := $(shell pwd)
CURRENT_DIR   := $(shell echo '$(CURRENT_DIR)' | sed 's/ /\\\ /g')

# C-compliant project name

PROJECT_NAME_AS_IDENTIFIER = embdx_VM

MAKEFILE_PATH  = $(CURRENT_DIR)/Makefiles
UTILITIES_PATH = $(CURRENT_DIR)/Utilities

include $(MAKEFILE_PATH)/Step1.mk

