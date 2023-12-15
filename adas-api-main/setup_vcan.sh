sudo modprobe can
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

canplayer -li -I docs/candump_vcu2ai.log vcan0=can0
