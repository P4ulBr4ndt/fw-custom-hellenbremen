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

# We want bluetooth
DDEFS += -DEFI_BLUETOOTH_SETUP=TRUE
DDEFS += $(PRIMARY_COMMUNICATION_PORT_USART2)