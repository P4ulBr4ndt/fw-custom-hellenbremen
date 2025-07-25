BOARDCPPSRC =  $(BOARD_DIR)/board_configuration.cpp \


BOARDINC += $(BOARD_DIR)/generated/controllers/generated

# defines SHORT_BOARD_NAME
include $(BOARD_DIR)/meta-info.env

# this includes knock and mux control settings
include $(BOARDS_DIR)/hellen/hellen-common-mega144.mk

# ACR Management
DDEFS += -DEFI_HD_ACR=TRUE

# We want to use ADC3 for knock sampling
DDEFS += -DSTM32_ADC_USE_ADC3=TRUE
DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE

# We want bluetooth and have maybe twisted rx/tx vs hellen default?
DDEFS += -DEFI_BLUETOOTH_SETUP=TRUE
DDEFS += -DEFI_CONSOLE_TX_BRAIN_PIN=Gpio::D5 -DEFI_CONSOLE_RX_BRAIN_PIN=Gpio::D6 -DTS_PRIMARY_UxART_PORT=UARTD2 -DSTM32_UART_USE_USART2=1