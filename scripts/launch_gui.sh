#!/bin/bash
#
# Usage:
#   launch_gui.sh <screen_size> <command> [arg1] [arg2] ...

screen_size=$1
shift

# Add a FreeDesktop 'autostart' desktop file which runs a given command
if [ $# -gt 0 ]; then
    mkdir -p ~/.config/autostart
    cat > ~/.config/autostart/autostart.desktop << _EOL_
[Desktop Entry]
Type=Application
Name=configures autostart application
NoDisplay=true
Exec=$*
Terminal=true
_EOL_
fi

exec x11vnc -forever -create \
    -env FD_PROG="lxsession -e LXDE -s Lubuntu" \
    -env FD_GEOM=${screen_size}
