BOARDCPPSRC =  $(BOARD_DIR)/board_configuration.cpp \


BOARDINC += $(BOARD_DIR)/generated/controllers/generated

# defines SHORT_BOARD_NAME
include $(BOARD_DIR)/meta-info.env

# this is usually used by Bluetooth
DDEFS += $(PRIMARY_COMMUNICATION_PORT_USART2)

# this includes knock and mux control settings
include $(BOARDS_DIR)/hellen/hellen-common-mega144.mk

# ACR Management
DDEFS += -DEFI_HD_ACR=TRUE

# This stuff doesn't work on H7 yet
ifneq ($(PROJECT_CPU),ARCH_STM32H7)
	DDEFS += -DSTM32_ADC_USE_ADC3=TRUE
	DDEFS += -DEFI_SOFTWARE_KNOCK=TRUE
endif