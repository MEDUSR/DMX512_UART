#!/bin/bash

# DMX512 Automatic Setup Script for Raspberry Pi 5
# This script is called by systemd on boot to configure the DMX interface

# Log function
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') DMX-SETUP: $1"
    logger -t dmx-setup "$1"
}

log "Starting DMX512 interface setup..."

# Check if DMX kernel module exists
DMX_MODULE="/lib/modules/$(uname -r)/extra/dmx_rpi5.ko"
if [ ! -f "$DMX_MODULE" ]; then
    log "ERROR: DMX kernel module not found at $DMX_MODULE"
    log "Run 'make install' in the DMX driver directory first"
    exit 1
fi

# Load the DMX kernel module with 512 active channels
log "Loading DMX kernel module..."
if modprobe dmx_rpi5 active_channels=512; then
    log "DMX kernel module loaded successfully with 512 active channels"
else
    log "ERROR: Failed to load DMX kernel module"
    exit 1
fi

# Wait for device to be created
sleep 1

# Check if DMX device was created
if [ ! -e "/dev/dmx0" ]; then
    log "ERROR: /dev/dmx0 device not found after module load"
    exit 1
fi

# Set permissions on DMX device
log "Setting permissions on /dev/dmx0..."
chmod 666 /dev/dmx0
chown root:dialout /dev/dmx0

# Configure GPIO 18 for DMX output mode (BitWizard requirement)
log "Configuring GPIO 18 for DMX output mode..."

# Use gpioset method (modern approach)
if command -v gpioset >/dev/null 2>&1; then
    # Set GPIO 18 HIGH for output mode and keep it active
    gpioset --mode=wait gpiochip0 18=1 &
    GPIO_PID=$!
    log "GPIO 18 set to HIGH using gpioset (PID: $GPIO_PID)"
else
    # Fallback to sysfs method
    GPIO_PIN=18
    GPIO_PATH="/sys/class/gpio/gpio$GPIO_PIN"
    
    # Export GPIO if not already exported
    if [ ! -d "$GPIO_PATH" ]; then
        echo $GPIO_PIN > /sys/class/gpio/export
        sleep 0.1
    fi
    
    # Set as output and HIGH
    echo out > $GPIO_PATH/direction
    echo 1 > $GPIO_PATH/value
    log "GPIO 18 set to HIGH using sysfs method"
fi

# Configure UART settings for DMX512 on ttyAMA0
log "Configuring UART settings for DMX512..."
UART_DEVICE="/dev/ttyAMA0"

if [ -e "$UART_DEVICE" ]; then
    # Configure UART: 250000 baud, 8 data bits, no parity, 2 stop bits, raw mode
    stty -F $UART_DEVICE 250000 cs8 cstopb -parenb raw -echo -echoe -echok -echoctl -echoke
    chmod 666 $UART_DEVICE
    log "UART $UART_DEVICE configured for DMX512 (250000 baud, 8N2, raw)"
else
    log "WARNING: $UART_DEVICE not found - check config.txt UART settings"
fi

# Verify setup
log "Verifying DMX setup..."
if [ -e "/dev/dmx0" ] && lsmod | grep -q dmx_rpi5; then
    log "DMX512 interface setup completed successfully"
    log "Device: /dev/dmx0"
    log "Command: dmx [channel] [value]"
    log "Stats: dmx stats"
    
    # Test basic functionality
    if command -v dmx >/dev/null 2>&1; then
        log "DMX command tool is available"
    else
        log "WARNING: dmx command tool not found in PATH"
    fi
    
    exit 0
else
    log "ERROR: DMX setup verification failed"
    exit 1
fi 
