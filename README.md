# odrive-config

## reconfigure the can interface
```bash
sudo ip link set down can0
sudo ip link set up can0 type can bitrate 250000
```

## show raw can message on console
```bash
candump can0
```