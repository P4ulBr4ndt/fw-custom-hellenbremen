BOARDCPPSRC =  $(BOARD_DIR)/board_configuration.cpp \


BOARDINC += $(BOARD_DIR)/generated/controllers/generated

# defines SHORT_BOARD_NAME
include $(BOARD_DIR)/meta-info.env

# this includes knock and mux control settings
include $(BOARDS_DIR)/hellen/hellen-common-mega144.mk

# We dont need this kind of board detection? 0xF000 + 99 = 61539
DDEFS += -DHW_HELLEN_SKIP_BOARD_TYPE=TRUE
DDEFS += -DHW_HELLEN_SKIP_BOARD_TYPE=61539

# Just making sure
DDEFS += -DADC_MUX_PIN=Gpio::F2

# ACR Management
DDEFS += -DEFI_HD_ACR=TRUE

# We want to use ADC3 for knock sampling
DDEFS += -DSTM32_ADC_USE_ADC3=TRUE
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE

# We want bluetooth and have maybe twisted rx/tx vs hellen default?
# Is there actually a problem with UART on F7?
DDEFS += -DEFI_CONSOLE_TX_BRAIN_PIN=Gpio::D5 -DEFI_CONSOLE_RX_BRAIN_PIN=Gpio::D6
DDEFS += -DTS_PRIMARY_UxART_PORT=SD2 -DEFI_TS_PRIMARY_IS_SERIAL=TRUE -DSTM32_SERIAL_USE_USART2=TRUE -DSTM32_UART_USE_USART2=FALSE

# Our custom code that only relates to our "special use cases"
DDEFS += -DEFI_HD_DP=TRUE