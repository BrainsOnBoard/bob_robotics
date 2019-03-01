#!/bin/bash
if [ $EUID -ne 0 ]; then
     sudo /bin/bash "$0" "$@" 
     exit $?
fi

loginctl enable-linger || return 1
install -m644 "$(dirname $0)"/ev3_control.service /etc/systemd/system/ || return 1
systemctl daemon-reload || return 1
systemctl enable --now ev3_control.service || return 1

echo Service installed and running
