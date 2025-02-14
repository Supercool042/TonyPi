#!/bin/bash
sleep 5
sudo raspi-config nonint do_expand_rootfs
sleep 3
sudo systemctl disable expand_rootfs.service
(sudo rpi-eeprom-config; echo "PSU_MAX_CURRENT=5000") | sudo tee /tmp/bootconf.txt
sudo rpi-eeprom-config --apply /tmp/bootconf.txt
sudo reboot

