####################################################################
# Makefile
#
# OS variable must either be 'posix' or 'win'. E.g. 'make OS=posix'.
# Error is thrown if OS variable is not equal with any of these.
#
####################################################################

.SUFFIXES:				# ignore builtin rules
.PHONY: all debug release clean export

####################################################################
# Definitions                                                      #
####################################################################

# uniq is a function which removes duplicate elements from a list
uniq = $(strip $(if $1,$(firstword $1) \
       $(call uniq,$(filter-out $(firstword $1),$1))))

PROJECTNAME = aoa_locator
CONFIG ?= default
SDK_DIR = ../../../..
OBJ_DIR = obj
EXE_DIR = exe
LST_DIR = lst
EXPORT_DIR = export

RTL_DIR = $(SDK_DIR)/util/silicon_labs/aox
JSON_DIR = $(SDK_DIR)/util/third_party/cjson

####################################################################
# Definitions of toolchain.                                        #
# You might need to do changes to match your system setup          #
####################################################################

RMDIRS     := rm -rf
RMFILES    := rm -rf
ALLFILES   := /*.*
NULLDEVICE := /dev/null
SHELLNAMES := $(ComSpec)$(COMSPEC)
UNAME      := $(shell uname | tr '[:upper:]' '[:lower:]')
DEVICE     := x64
ifneq ($(filter arm%, $(shell uname -m)),)
DEVICE     := cortexa
endif

# Try to detect NULL device regardless of the environment we are running on.
ifeq (,$(wildcard $(NULLDEVICE)))
  NULLDEVICE := NUL
endif

ifeq (export,$(findstring export, $(MAKECMDGOALS)))
  # Set the default OS for exporting if not specified externally
  ifeq (,$(filter $(OS),posix win))
    OS:=posix
  endif
else
  # Try autodetecting the environment: Windows
  ifneq ($(SHELLNAMES),)
    QUOTE :="
    ifeq (,$(filter $(OS),posix win))
      OS:=win
    endif
    ifneq ($(COMSPEC),)
      ifeq ($(findstring cygdrive,$(shell set)),)
        # We were not on a cygwin platform
        # MSYS platform. Override environment here.

      endif
    else
      # Assume we are making on a Windows platform
      # This is a convenient place to override TOOLDIR, DO NOT add trailing
      # whitespace chars, they do matter !
      SHELL      := $(SHELLNAMES)
      RMDIRS     := rd /s /q
      RMFILES    := del /s /q
      ALLFILES   := \*.*
    endif
  # Other than Windows
  else
    ifeq (,$(filter $(OS),posix win))
      OS:=posix
    endif
  endif
endif

# Create directories and do a clean which is compatible with parallell make
$(shell mkdir $(OBJ_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(EXE_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(LST_DIR)>$(NULLDEVICE) 2>&1)
ifeq (clean,$(findstring clean, $(MAKECMDGOALS)))
  ifneq ($(filter $(MAKECMDGOALS),all debug release),)
    $(shell $(RMFILES) $(OBJ_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(EXE_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(LST_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
  endif
endif

ifeq ($(OS),posix)
CC = gcc
LD = ld
AR = ar
else
CC = x86_64-w64-mingw32-gcc
LD = x86_64-w64-mingw32-ld
AR = x86_64-w64-mingw32-ar
endif


####################################################################
# Flags                                                            #
####################################################################

INCLUDEPATHS += . \
$(SDK_DIR)/app/bluetooth/common_host/uart \
$(SDK_DIR)/app/bluetooth/common_host/tcp \
$(SDK_DIR)/app/bluetooth/common_host/system \
$(SDK_DIR)/app/bluetooth/common_host/app_assert \
$(SDK_DIR)/app/bluetooth/common_host/app_signal \
$(SDK_DIR)/app/bluetooth/common_host/app_log \
$(SDK_DIR)/app/bluetooth/common_host/app_log/config \
$(SDK_DIR)/app/bluetooth/common/ncp_evt_filter \
$(SDK_DIR)/protocol/bluetooth/inc \
$(SDK_DIR)/platform/common/inc \
$(JSON_DIR) \
$(RTL_DIR)/inc \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util \
$(SDK_DIR)/app/bluetooth/common_host/aoa_config/$(CONFIG) \
$(SDK_DIR)/app/bluetooth/common_host/mqtt

INCFLAGS = $(addprefix -I, $(INCLUDEPATHS))

# make can't deal with spaces in paths, add mosquitto path separately
ifeq ($(OS),win)
INCFLAGS += -I"$(MOSQUITTO_DIR)/devel"
endif

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = \
-MMD \
-MP \
-MF $(@:.o=.d)

# Add -Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
override CFLAGS += \
-fno-short-enums \
-Wall \
-c \
-fmessage-length=0 \
-std=c99 \
$(DEPFLAGS)

# Linux platform: if _DEFAULT_SOURCE is defined, the default is to have _POSIX_SOURCE set to one
# and _POSIX_C_SOURCE set to 200809L, as well as enabling miscellaneous functions from BSD and SVID.
# See usr/include/fetures.h for more information.
# 
# _BSD_SOURCE (deprecated since glibc 2.20)
# Defining this macro with any value causes header files to expose BSD-derived definitions.
# In glibc versions up to and including 2.18, defining this macro also causes BSD definitions to be
# preferred in some situations where standards conflict, unless one or more of _SVID_SOURCE,
# _POSIX_SOURCE, _POSIX_C_SOURCE, _XOPEN_SOURCE, _XOPEN_SOURCE_EXTENDED, or _GNU_SOURCE is defined,
# in which case BSD definitions are disfavored. Since glibc 2.19, _BSD_SOURCE no longer causes BSD
# definitions to be preferred in case of conflicts. Since glibc 2.20, this macro is deprecated. 
# It now has the same effect as defining _DEFAULT_SOURCE, but generates a compile-time warning
# (unless _DEFAULT_SOURCE is also defined). Use _DEFAULT_SOURCE instead.
# To allow code that requires _BSD_SOURCE in glibc 2.19 and earlier and _DEFAULT_SOURCE in glibc
# 2.20 and later to compile without warnings, define both _BSD_SOURCE and _DEFAULT_SOURCE.
#
# OSX platform: _DEFAULT_SOURCE is not used, instead _DARWIN_C_SOURCE is defined by default.
ifeq ($(OS),posix)
override CFLAGS += \
-D_DEFAULT_SOURCE \
-D_BSD_SOURCE
endif

# NOTE: The -Wl,--gc-sections flag may interfere with debugging using gdb.
ifeq ($(OS),posix)
override LDFLAGS += \
-L$(RTL_DIR)/lib/$(UNAME)_$(DEVICE)/gcc/release \
-laox_static \
-lmosquitto \
-lstdc++ \
-lpthread \
-lm
else
override LDFLAGS += \
-static \
"$(RTL_DIR)/lib/windows_x64/gcc/release/libaox_static.a" \
"${MOSQUITTO_DIR}/devel/mosquitto.lib" \
-lstdc++ \
-lpthread \
-lWs2_32
endif


####################################################################
# Files                                                            #
####################################################################

C_SRC +=  \
$(SDK_DIR)/app/bluetooth/common_host/system/system.c \
$(SDK_DIR)/protocol/bluetooth/src/sl_bt_ncp_host.c \
$(SDK_DIR)/protocol/bluetooth/src/sl_bt_ncp_host_api.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_util.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_parse.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_util/aoa_serdes.c \
$(SDK_DIR)/app/bluetooth/common_host/aoa_config/$(CONFIG)/aoa_config.c \
$(JSON_DIR)/cJSON.c \
$(SDK_DIR)/app/bluetooth/common_host/app_signal/app_signal_$(OS).c \
$(SDK_DIR)/app/bluetooth/common_host/mqtt/mqtt.c \
app.c \
aoa.c \
conn.c \
main.c

ifeq (${APP_MODE},conn_less)
C_SRC += app_conn_less.c
else ifeq (${APP_MODE},silabs)
C_SRC += app_silabs.c
else ifeq (${APP_MODE},conn)
C_SRC += app_conn.c
else
C_SRC += app_silabs.c
endif

# this file should be the last added
C_SRC += \
$(SDK_DIR)/app/bluetooth/common_host/uart/uart_$(OS).c \
$(SDK_DIR)/app/bluetooth/common_host/tcp/tcp_$(OS).c

ifeq ($(OS),posix)
LIBS =
else
LIBS = \
$(EXE_DIR)/mosquitto.dll \
$(EXE_DIR)/libcrypto-1_1-x64.dll \
$(EXE_DIR)/libssl-1_1-x64.dll
endif

# Project resources
INC_FILES = $(foreach dir,$(INCLUDEPATHS),$(wildcard $(dir)/*.h))
PROJ_FILES = $(C_SRC) $(INC_FILES) $(RTL_DIR)/lib makefile
DST_DIR = $(EXPORT_DIR)/app/bluetooth/example_host/$(PROJECTNAME)/
DST_FILES := $(addprefix $(DST_DIR), $(PROJ_FILES))


####################################################################
# Rules                                                            #
####################################################################

C_FILES = $(notdir $(C_SRC) )
#make list of source paths, uniq removes duplicate paths
C_PATHS = $(call uniq, $(dir $(C_SRC) ) )

C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
OBJS = $(C_OBJS)

vpath %.c $(C_PATHS)

# Default build is debug build
all:      debug

debug:    CFLAGS += -O0 -g3
debug:    $(EXE_DIR)/$(PROJECTNAME)

release:  $(EXE_DIR)/$(PROJECTNAME)


# Create objects from C SRC files
$(OBJ_DIR)/%.o: %.c
	@echo "Building file: $<"
	$(CC) $(CFLAGS) $(INCFLAGS) -c -o $@ $<

# Link
$(EXE_DIR)/$(PROJECTNAME): $(OBJS) $(LIBS)
	@echo "Linking target: $@"
	$(CC) $^ $(LDFLAGS) -o $@

# Copy .dll files (Windows only)
$(EXE_DIR)/%.dll:
	$(shell cp "${MOSQUITTO_DIR}/$*.dll" $(EXE_DIR))

clean:
ifeq ($(filter $(MAKECMDGOALS),all debug release),)
	$(RMDIRS) $(OBJ_DIR) $(LST_DIR) $(EXE_DIR) $(EXPORT_DIR)
endif

# Collect project files for exporting
$(DST_FILES) : $(addprefix $(DST_DIR), %) : %
	@mkdir -p $(dir $@) && cp -pRv $< $@

export: $(DST_FILES)
	@echo "Exporting done."

# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
endif
