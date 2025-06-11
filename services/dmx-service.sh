#!/bin/bash
# DMX512 Service Management Script

SERVICE_NAME="dmx-autostart.service"

case "$1" in
    status)
        echo "🔍 DMX Service Status:"
        sudo systemctl status $SERVICE_NAME
        ;;
    enable)
        echo "✅ Enabling DMX autostart..."
        sudo systemctl enable $SERVICE_NAME
        echo "DMX will now start automatically on boot"
        ;;
    disable)
        echo "❌ Disabling DMX autostart..."
        sudo systemctl disable $SERVICE_NAME
        echo "DMX will no longer start automatically on boot"
        ;;
    start)
        echo "🚀 Starting DMX service..."
        sudo systemctl start $SERVICE_NAME
        ;;
    stop)
        echo "🛑 Stopping DMX service..."
        sudo systemctl stop $SERVICE_NAME
        ;;
    restart)
        echo "🔄 Restarting DMX service..."
        sudo systemctl restart $SERVICE_NAME
        ;;
    logs)
        echo "📋 DMX Service Logs:"
        sudo journalctl -u $SERVICE_NAME -f
        ;;
    install)
        echo "📦 Installing DMX autostart service..."
        sudo cp dmx-autostart.service /etc/systemd/system/
        sudo systemctl daemon-reload
        sudo systemctl enable $SERVICE_NAME
        echo "✅ Service installed and enabled"
        ;;
    uninstall)
        echo "🗑️ Uninstalling DMX autostart service..."
        sudo systemctl stop $SERVICE_NAME
        sudo systemctl disable $SERVICE_NAME
        sudo rm /etc/systemd/system/$SERVICE_NAME
        sudo systemctl daemon-reload
        echo "✅ Service uninstalled"
        ;;
    *)
        echo "🎭 DMX512 Service Manager"
        echo "========================"
        echo "Usage: $0 {status|enable|disable|start|stop|restart|logs|install|uninstall}"
        echo ""
        echo "Commands:"
        echo "  status    - Show service status"
        echo "  enable    - Enable autostart on boot"
        echo "  disable   - Disable autostart on boot"
        echo "  start     - Start service now"
        echo "  stop      - Stop service now"
        echo "  restart   - Restart service"
        echo "  logs      - Show service logs (press Ctrl+C to exit)"
        echo "  install   - Install the service"
        echo "  uninstall - Remove the service"
        exit 1
        ;;
esac 
