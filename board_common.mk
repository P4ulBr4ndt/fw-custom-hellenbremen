# Included by the simulator build (see ext/rusefi/simulator/Makefile).
# The firmware build already puts BOARD_DIR on the include path via BOARDINC;
# the simulator does not, so <board_types.h> would resolve to the empty core
# placeholder and the generated config structs would miss board enums
# such as autotuneCellChangeResistance_e.
INCDIR += $(BOARD_DIR)
