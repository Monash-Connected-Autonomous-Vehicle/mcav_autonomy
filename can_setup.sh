sudo modprobe peak_usb && sudo ip link set can0 up type can bitrate 500000

# Check if the previous command was successful
if [ $? -eq 0 ]
then
    echo "CAN bus setup successful"
else
    echo "Error. CAN bus setup failed. Disconnect and try again"
fi
