sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control –reload-rules && udevadm trigger
