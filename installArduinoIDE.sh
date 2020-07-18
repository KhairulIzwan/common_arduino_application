#!/bin/bash
# The code originall courtesy of jetsonhacks.com
# Re-arrange to be use with Raspberry Pi 4 (Raspbian Buster or later) 

INSTALL_DIR=${HOME}
# You may check current (latest IDE)
ARDUINO_VERSION=1.8.12

# Only download if newer version exists
wget -N https://downloads.arduino.cc/arduino-$ARDUINO_VERSION-linuxarm.tar.xz
tar -C $INSTALL_DIR/ -xvf arduino-${ARDUINO_VERSION}-linuxarm.tar.xz
cd $INSTALL_DIR/arduino-${ARDUINO_VERSION}
sudo ./install.sh
./arduino-linux-setup.sh "$USER"
echo "You can delete the tar file if desired: arduino-"${ARDUINO_VERSION}"-linuxarm.tar.xz"

echo "Enjoy coding!"s
