#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := zauberzeug-robot-brain
EXTRA_COMPONENT_DIRS += $(PROJECT_PATH)/src

include $(IDF_PATH)/make/project.mk
