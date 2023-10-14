#!/bin/sh

ssh -t tegra-ubuntu.local -C "sudo date -s \"$(date)\""
