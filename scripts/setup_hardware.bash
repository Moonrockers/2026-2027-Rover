#!/usr/bin/env bash

# setup canbus
sudo ip link set can0 down >> /dev/null
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 txqueuelen 1000 
sudo ip link set can0 up type can bitrate 125000 restart-ms 100