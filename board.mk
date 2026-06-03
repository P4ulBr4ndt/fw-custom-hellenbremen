ifneq ($(findstring EFI_BOOTLOADER,$(DDEFS)),)
  # Bootloader build: skip app-only board configuration
  BOARDCPPSRC =
else
  BOARDCPPSRC = $(BOARD_DIR)/board_configuration.cpp \
                $(BOARD_DIR)/board_config.cpp \
                $(BOARD_DIR)/board_can.cpp \
                $(BOARD_DIR)/board_riding_modes.cpp \
                $(BOARD_DIR)/board_etb_maps.cpp \
                $(BOARD_DIR)/board_instant_accel_shot.cpp \
                $(BOARD_DIR)/board_uds.cpp
endif

BOARDINC += $(BOARD_DIR)/generated/controllers/generated

# defines SHORT_BOARD_NAME
include $(BOARD_DIR)/meta-info.env

# this includes knock and mux control settings
include $(BOARDS_DIR)/hellen/hellen-common-mega144.mk

# We dont need this kind of board detection? 0xF000 + 99 = 61539
DDEFS += -DHW_HELLEN_SKIP_BOARD_TYPE=TRUE
DDEFS += -DSTATIC_BOARD_ID=61539

# Just making sure
DDEFS += -DADC_MUX_PIN=Gpio::F2

# ACR Management
DDEFS += -DEFI_HD_ACR=TRUE
DDEFS += -DEFI_HD_ADVANCED_ACR=TRUE
DDEFS += -DEFI_HD_ADVANCED_ACR_DEBUG=FALSE

# We want to use ADC3 for knock sampling
DDEFS += -DSTM32_ADC_USE_ADC3=TRUE
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE

# We want bluetooth and have maybe twisted rx/tx vs hellen default?
# Is there actually a problem with UART on F7?
DDEFS += -DEFI_CONSOLE_TX_BRAIN_PIN=Gpio::D5 -DEFI_CONSOLE_RX_BRAIN_PIN=Gpio::D6
DDEFS += -DTS_PRIMARY_UxART_PORT=SD2 -DEFI_TS_PRIMARY_IS_SERIAL=TRUE -DSTM32_SERIAL_USE_USART2=TRUE -DSTM32_UART_USE_USART2=FALSE

# Our custom code that only relates to our "special use cases"
DDEFS += -DEFI_HD_DP=TRUE

# Store LTFT in internal flash (not SD storage backend)
DDEFS += -DEFI_STORAGE_SD=FALSE

# USB product string shown by host OS
DDEFS += -DUSB_DESCRIPTOR_STRING_CONTENT="'B', 0, 'r', 0, 'e', 0, 'm', 0, 'e', 0, 'r', 0, 'E', 0, 'C', 0, 'U', 0"

# OpenBLT CAN IDs (standard 11-bit)
DDEFS += -DBOOT_COM_CAN_RX_MSG_ID=0x7E0 -DBOOT_COM_CAN_TX_MSG_ID=0x7E8 -DBOOT_COM_CAN_EXT=0
# OpenBLT CAN physical pins: match main firmware CAN1 (PD0/PD1)
DDEFS += -DBOOT_COM_CAN_CHANNEL_INDEX=0
DDEFS += -DOPENBLT_CAN_RX_PORT=GPIOD -DOPENBLT_CAN_RX_PIN=0
DDEFS += -DOPENBLT_CAN_TX_PORT=GPIOD -DOPENBLT_CAN_TX_PIN=1

# Keep TunerStudio-over-CAN ISO-TP frames padded to classic CAN DLC 8.
DDEFS += -DTSCAN_ALWAYS_DLC_8=TRUE

# let's drop some fancy lua functions
DDEFS += -DWITH_LUA_CONSUMPTION=FALSE
DDEFS += -DWITH_LUA_PID=FALSE
DDEFS += -DWITH_LUA_STOP_ENGINE=FALSE
