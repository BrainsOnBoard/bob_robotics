#!/bin/bash
if [ $EUID -eq 0 ]; then
     echo Error: This script should not be run as root > /dev/stderr
     exit 1
fi

sudo install -m644 "$(dirname $0)"/tank_control@.service /etc/systemd/system/ || exit 1
sudo systemctl daemon-reload || exit 1
sudo systemctl enable --now tank_control@$USER || exit 1

echo Service installed and running
