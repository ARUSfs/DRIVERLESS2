#!/bin/bash
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 1000000

candump can0 >> ~/candumps/can0_dump_$(date +'%Y-%m-%d_%H-%M-%S').txt &
candump can1 >> ~/candumps/can1_dump_$(date +'%Y-%m-%d_%H-%M-%S').txt

sudo timedatectl set-local-rtc 1