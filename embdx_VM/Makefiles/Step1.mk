
# Board selection
# ----------------------------------
# Board specifics defined in .xconfig file
# BOARD_TAG and AVRDUDE_PORT 
#
ifneq ($(MAKECMDGOALS),boards)
    ifneq ($(MAKECMDGOALS),clean)
        ifndef BOARD_TAG
            $(error BOARD_TAG not defined)
        endif
    endif
endif

ifndef BOARD_PORT
    BOARD_PORT = /dev/tty.usb*
endif


# Path to applications folder
#
USER_PATH      := $(wildcard ~)
EMBEDXCODE_APP  = $(USER_PATH)/Library/embedXcode
PARAMETERS_TXT  = $(EMBEDXCODE_APP)/parameters.txt

ifndef APPLICATIONS_PATH
    APPLICATIONS_PATH = /Applications
endif

# APPlications full paths
# ----------------------------------
#
# Welcome dual releases 1.6.1 and 1.7 with new and fresh nightmares!
# Arduino, ArduinoCC and ArduinoORG
#
ifneq ($(wildcard $(APPLICATIONS_PATH)/Arduino.app),)
    ARDUINO_APP   := $(APPLICATIONS_PATH)/Arduino.app
else ifneq ($(wildcard $(APPLICATIONS_PATH)/ArduinoCC.app),)
    ARDUINO_APP   := $(APPLICATIONS_PATH)/ArduinoCC.app
else ifneq ($(wildcard $(APPLICATIONS_PATH)/ArduinoORG.app),)
    ARDUINO_APP   := $(APPLICATIONS_PATH)/ArduinoORG.app
endif





# Arduino-related nightmares
# ----------------------------------

ifneq ($(wildcard $(ARDUINO_APP)),) # */
    s102 = $(ARDUINO_APP)/Contents/Resources/Java/lib/version.txt
    s103 = $(ARDUINO_APP)/Contents/Java/lib/version.txt
    ifneq ($(wildcard $(s102)),)
        ARDUINO_RELEASE := $(shell cat $(s102) | sed -e "s/\.//g")
    else
        ARDUINO_RELEASE := $(shell cat $(s103) | sed -e "s/\.//g")
    endif
    ARDUINO_MAJOR := $(shell echo $(ARDUINO_RELEASE) | cut -d. -f 1-2)
else
    ARDUINO_RELEASE := 0
    ARDUINO_MAJOR   := 0
endif

# But nightmare continues with 2 releases for Arduino 1.6
# Different folder locations in Java 6 and Java 7 versions
# Another example of Arduino's quick and dirty job :(
#
ifeq ($(wildcard $(ARDUINO_APP)/Contents/Resources/Java),)
    ARDUINO_PATH   := $(ARDUINO_APP)/Contents/Java
else
    ARDUINO_PATH   := $(ARDUINO_APP)/Contents/Resources/Java
endif


# Miscellaneous
# ----------------------------------
# Variables
#
TARGET      := embeddedcomputing
USER_FLAG   := false
TEMPLATE    := ePsiEJEtRXnDNaFGpywBX9vzeNQP4vUb

# Builds directory
#
OBJDIR  = Builds

# Function PARSE_BOARD data retrieval from boards.txt
# result = $(call PARSE_BOARD 'boardname','parameter')
#
PARSE_BOARD = $(shell if [ -f $(BOARDS_TXT) ]; then grep ^$(1).$(2)= $(BOARDS_TXT) | cut -d = -f 2-; fi; )

# Function PARSE_FILE data retrieval from specified file
# result = $(call PARSE_FILE 'boardname','parameter','filename')
#
PARSE_FILE = $(shell if [ -f $(3) ]; then grep ^$(1).$(2) $(3) | cut -d = -f 2-; fi; )


# Clean if new BOARD_TAG
# ----------------------------------
#
NEW_TAG := $(strip $(OBJDIR)/$(BOARD_TAG)-TAG) #
OLD_TAG := $(strip $(wildcard $(OBJDIR)/*-TAG)) # */

ifneq ($(OLD_TAG),$(NEW_TAG))
    CHANGE_FLAG := 1
else
    CHANGE_FLAG := 0
endif


# Identification and switch
# ----------------------------------
# Look if BOARD_TAG is listed as a Arduino/Arduino board
# Look if BOARD_TAG is listed as a Arduino/arduino/avr board *1.5

ifneq ($(MAKECMDGOALS),boards)
    ifneq ($(MAKECMDGOALS),clean)

        # ArduinoCC
        ifneq ($(call PARSE_FILE,$(BOARD_TAG),name,$(ARDUINO_PATH)/hardware/arduino/boards.txt),)
            include $(MAKEFILE_PATH)/Arduino.mk
        else ifneq ($(call PARSE_FILE,$(BOARD_TAG),name,$(ARDUINO_PATH)/hardware/arduino/avr/boards.txt),)
            include $(MAKEFILE_PATH)/Arduino15avr.mk
        else ifneq ($(call PARSE_FILE,$(BOARD_TAG1),name,$(ARDUINO_PATH)/hardware/arduino/avr/boards.txt),)
            include $(MAKEFILE_PATH)/Arduino15avr.mk
        else ifneq ($(call PARSE_FILE,$(BOARD_TAG),name,$(ARDUINO_PATH)/hardware/arduino/sam/boards.txt),)
            include $(MAKEFILE_PATH)/Arduino15sam.mk
        else ifneq ($(call PARSE_FILE,$(BOARD_TAG),name,$(ARDUINO_PATH)/hardware/arduino/boards.txt),)
            include $(MAKEFILE_PATH)/Arduino.mk

        else
            $(error $(BOARD_TAG) board is unknown)
        endif
    endif
endif


# List of sub-paths to be excluded
#
EXCLUDE_NAMES  = Example example Examples examples Archive archive Archives archives Documentation documentation Reference reference
EXCLUDE_NAMES += ArduinoTestSuite
EXCLUDE_NAMES += $(EXCLUDE_LIBS)
EXCLUDE_LIST   = $(addprefix %,$(EXCLUDE_NAMES))

# Step 2
#
include $(MAKEFILE_PATH)/Step2.mk

