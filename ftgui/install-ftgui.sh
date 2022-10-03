#!/bin/bash
sudo apt-get install debhelper libgtk2.0-dev libgtk2.0-bin libglade2-dev autotools-dev byacc flex texinfo docbook-to-man libvte-dev gcc-8 g++-8 cpp-8 git
git clone http://github.com/bm16ton/gtkdialog
cp switch* gtkdialog
cd gtkdialog
./switch-to-gcc8.sh
./configure --enable-gtk3
make -j$(nproc)
sudo cp src/gtkdialog /usr/local/bin/
./switch-gcc-back.sh
cd ..
sed -i 's/maddocks/'"$USER"'/g' ft232h-app
sudo cp ft232h* /usr/local/bin
sudo chmod +x /usr/local/bin/ft232h*
sudo cp mpssegui.service /lib/systemd/system/
sudo systemctl enable --now mpssegui.service
sudo cp 99-ftdimpsse.rules /etc/udev/rules.d
sudo udevadm control --reload
