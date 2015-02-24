#!/bin/sh
###############################################################################
#J.S. PEIRCE
#CAPSTONE 2015
#EREBUS LABS - SENSE PLATFORM
#24 FEBRUARY 2015
###############################################################################
# This script is for starting an OpenOCD session for programming or debugging
#
# Make sure you have followed the proper steps for connecting via JTAG
# 1 - Start with power to the target turned off
# 2 - Verify that the JTAG connecter is right
# 3 - Verify cable orientation
# 4 - Connect the adapter's other end after connecting the target board
# 5 - Connect the adapter's power supply
# 6 - Power up the target board

INTERFACE=olimex-arm-usb-ocd-h
LOG_NAME=jtag.log
LOG_LEVEL=3 # 0 is lowest, 3 is highest
TARGET=stm32f2x

# Ensure this script is run as root for the OpenOCD session
if [[ $EUID != "0" ]]; then
	echo "This script must be run as root"
	exit 1
fi

# Call init command?
#openocd -d $LOG_LEVEL -l $LOG_NAME -f interface/ftdi/olimex-arm-ocd-h.cfg -f target/stm32f2x.cfg
openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f target/stm32f2x.cfg -f commands.cfg

exit 0

# vim: set syntax=sh:
