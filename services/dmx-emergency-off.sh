#!/bin/bash

# DMX Emergency Safety Shutdown Script
# This script safely turns off all DMX channels in emergency situations

# Configuration
DMX_DEVICE="/dev/dmx0"
DMX_CMD="/usr/local/bin/dmx"
LOGFILE="/var/log/dmx-safety.log"

# Log function
log_event() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') DMX-SAFETY: $1" | tee -a "$LOGFILE"
    logger -t dmx-safety "$1"
}

# Check if DMX device exists
check_dmx_device() {
    if [ ! -e "$DMX_DEVICE" ]; then
        log_event "ERROR: DMX device $DMX_DEVICE not found"
        return 1
    fi
    return 0
}

# Emergency shutdown function
emergency_shutdown() {
    local reason="${1:-Unknown reason}"
    
    log_event "EMERGENCY SHUTDOWN INITIATED: $reason"
    
    # Check if DMX device is available
    if ! check_dmx_device; then
        log_event "Cannot perform emergency shutdown - DMX device unavailable"
        return 1
    fi
    
    # Turn off all channels
    if "$DMX_CMD" off >/dev/null 2>&1; then
        log_event "SUCCESS: All DMX channels turned off safely"
        return 0
    else
        log_event "ERROR: Failed to turn off DMX channels"
        # Try alternative method - reset through kernel
        if "$DMX_CMD" reset >/dev/null 2>&1; then
            log_event "SUCCESS: DMX channels reset as fallback"
            return 0
        else
            log_event "CRITICAL: Failed to turn off DMX channels - manual intervention required"
            return 1
        fi
    fi
}

# Power monitoring function (if hardware supports it)
monitor_power() {
    log_event "Starting power monitoring (if available)"
    
    # Check for UPS monitoring tools
    if command -v upsc >/dev/null 2>&1; then
        log_event "UPS monitoring detected - setting up power loss detection"
        # This would require UPS configuration
        # For now, just log that it's available
    fi
    
    # Monitor GPIO for power detection (if wired)
    # GPIO 3 is commonly used for power detection
    if [ -d "/sys/class/gpio/gpio3" ]; then
        log_event "GPIO power monitoring available on GPIO 3"
    fi
}

# Main execution
case "${1:-shutdown}" in
    "power-loss")
        emergency_shutdown "Power loss detected"
        ;;
    "shutdown")
        emergency_shutdown "System shutdown"
        ;;
    "emergency")
        emergency_shutdown "Manual emergency activation"
        ;;
    "test")
        log_event "Testing emergency shutdown system"
        emergency_shutdown "Safety system test"
        ;;
    "monitor")
        monitor_power
        ;;
    *)
        echo "DMX Emergency Safety System"
        echo "Usage: $0 [power-loss|shutdown|emergency|test|monitor]"
        echo ""
        echo "  power-loss : Shutdown due to power loss"
        echo "  shutdown   : Normal system shutdown (default)"
        echo "  emergency  : Manual emergency activation"
        echo "  test       : Test the safety system"
        echo "  monitor    : Start power monitoring"
        ;;
esac 
