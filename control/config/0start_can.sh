#!/bin/bash

echo "==> Subindo interface CAN0 com bitrate 500000"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

ip link show can0
