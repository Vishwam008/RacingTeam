#!/usr/bin/env bash

sudo modprobe can 
sudo ip link set up can0 type can bitrate 500000 
# sudo ip link set up can0 type can bitrate 1000000 
