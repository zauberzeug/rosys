#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

CPPFLAGS += -D_GLIBCXX_USE_C99 # https://github.com/espressif/esp-idf/issues/1445#issuecomment-354456491
COMPONENT_SRCDIRS := . modules/*