#------------------------------------------------------------------------------
# Firmware build
#
# Selectable build options
#------------------------------------------------------------------------------

#TARGET_BOARD         ?= BOARD_PCA10000
#TARGET_BOARD         ?= BOARD_PCA10001
TARGET_BOARD         ?= BOARD_PCA10028
#TARGET_BOARD         ?= BOARD_PCA10031

USE_RBC_MESH_SERIAL  = "yes"

#------------------------------------------------------------------------------
# Define relative paths to SDK components
#------------------------------------------------------------------------------

SDK_BASE      := ../../../../..
COMPONENTS    := $(SDK_BASE)/components
TEMPLATE_PATH := $(COMPONENTS)/toolchain/gcc

LINKER_SCRIPT := ./gcc_nrf51_s110_xxac.ld

ifeq ($(USE_RBC_MESH_SERIAL), "yes")
  SERIAL_STRING="_serial"
else
  SERIAL_STRING=""
endif

OUTPUT_NAME := bootloader$(SERIAL_STRING)_$(TARGET_BOARD)

#------------------------------------------------------------------------------
# Proceed cautiously beyond this point.  Little should change.
#------------------------------------------------------------------------------

export OUTPUT_NAME
export GNU_INSTALL_ROOT

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

ifeq ($(OS),Windows_NT)
  include $(TEMPLATE_PATH)/Makefile.windows
else
  include $(TEMPLATE_PATH)/Makefile.posix
endif

# echo suspend
ifeq ("$(VERBOSE)","1")
  NO_ECHO :=
else
  NO_ECHO := @
endif

ifeq ($(MAKECMDGOALS),debug)
  BUILD_TYPE := debug
else
  BUILD_TYPE := release
endif

# Toolchain commands
CC       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE     := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"
MK       := mkdir
RM       := rm -rf
CP       := cp

# function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

# source common to all targets

C_SOURCE_FILES += ../main.c
C_SOURCE_FILES += ../bootloader_info.c
C_SOURCE_FILES += ../bootloader_mesh.c
C_SOURCE_FILES += ../bootloader_rtc.c
C_SOURCE_FILES += ../dfu_mesh.c
C_SOURCE_FILES += ../transport.c
C_SOURCE_FILES += micro-ecc/uECC.c

ifeq ($(USE_RBC_MESH_SERIAL), "yes")
	CFLAGS += -D SERIAL=1
	C_SOURCE_FILES += ../../../rbc_mesh/src/serial_handler_uart.c
	C_SOURCE_FILES += ../../../rbc_mesh/src/mesh_aci.c
endif

C_SOURCE_FILES += ../../rbc_mesh/src/radio_control.c
C_SOURCE_FILES += ../../rbc_mesh/src/fifo.c
C_SOURCE_FILES += ../../rbc_mesh/src/mesh_packet.c
C_SOURCE_FILES += ../../rbc_mesh/src/bootloader_util.c
C_SOURCE_FILES += ../../rbc_mesh/src/rand.c
C_SOURCE_FILES += ../../rbc_mesh/src/nrf_flash.c


C_SOURCE_FILES += $(COMPONENTS)/toolchain/system_nrf51.c
C_SOURCE_FILES += $(COMPONENTS)/libraries/sha256/sha256.c

# assembly files common to all targets
ASM_SOURCE_FILES  += $(COMPONENTS)/toolchain/gcc/gcc_startup_nrf51.s

# includes common to all targets

INC_PATHS += -I../include
INC_PATHS += -Imicro-ecc
INC_PATHS += -I../../rbc_mesh
INC_PATHS += -I../../rbc_mesh/include
INC_PATHS += -I../../SDK/bsp

INC_PATHS += -I$(COMPONENTS)/device
INC_PATHS += -I$(COMPONENTS)/toolchain
INC_PATHS += -I$(COMPONENTS)/toolchain/gcc
INC_PATHS += -I$(COMPONENTS)/libraries/util
INC_PATHS += -I$(COMPONENTS)/libraries/sha256
INC_PATHS += -I$(COMPONENTS)/drivers_nrf/hal
INC_PATHS += -I$(COMPONENTS)/softdevice/s110/headers

OBJECT_DIRECTORY := _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

ifeq ($(BUILD_TYPE),debug)
  DEBUG_FLAGS += -D DEBUG -g -O0
else
  DEBUG_FLAGS += -D NDEBUG -Os
endif

# flags common to all targets
#CFLAGS += -save-temps
CFLAGS += $(DEBUG_FLAGS)
CFLAGS += -D NRF51
CFLAGS += -D $(TARGET_BOARD)
CFLAGS += -D BOOTLOADER
CFLAGS += -D RBC_MESH_DATA_CACHE_ENTRIES=32
CFLAGS += -D uECC_CURVE=uECC_secp256r1
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror
CFLAGS += -Wa,-adhln
CFLAGS += -mfloat-abi=soft
CFLAGS += -fdata-sections -ffunction-sections -fno-strict-aliasing
CFLAGS += -fno-builtin
CFLAGS += -flto

# Linker flags
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_NAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
LDFLAGS += $(DEBUG_FLAGS)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += $(DEBUG_FLAGS)
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D NRF51
ASMFLAGS += -D $(TARGET_BOARD)

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

TOOLCHAIN_BASE = $(basename $(notdir $(GNU_INSTALL_ROOT)))

TIMESTAMP := $(shell date +'%s')

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

all: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_NAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

	@echo "*****************************************************"
	@echo "build project: $(OUTPUT_NAME)"
	@echo "build type:    $(BUILD_TYPE)"
	@echo "build with:    $(TOOLCHAIN_BASE)"
	@echo "build target:  $(TARGET_BOARD)"
	@echo "build options  --"
	@echo "               USE_RBC_MESH_SERIAL $(USE_RBC_MESH_SERIAL)"
	@echo "build products --"
	@echo "               $(OUTPUT_NAME).elf"
	@echo "               $(OUTPUT_NAME).hex"
	@echo "*****************************************************"

debug : all

release : all

# Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) \
	-c $< -o $@ > $(OUTPUT_BINARY_DIRECTORY)/$*.lst

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_NAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf

# Create binary .bin file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	@echo Preparing: $(OUTPUT_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin

# Create binary .hex file from the .elf file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	@echo Preparing: $(OUTPUT_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).bin

# Create binary .hex file from the .elf file
genhex:
	@echo Preparing: $(OUTPUT_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_NAME).elf
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

