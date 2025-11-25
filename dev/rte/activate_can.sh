
Baudrate=1000000

sudo ip link set can0 type can bitrate $Baudrate
sudo ip link set up can0

echo "CAN0 is up"
