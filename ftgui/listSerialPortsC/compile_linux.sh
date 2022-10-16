#!/bin/bash -xe

JAVA_INCLUDE_PATH=/usr/lib/jvm/java-8-openjdk-amd64/include

mkdir -p distrib/linux64
cd libserialport
./autogen.sh
./configure
make clean
make
cd ..
gcc main.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -Ilibserialport/  -o listSerialC
cp listSerialC distrib/linux64/listSerialC
gcc jnilib.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -Ilibserialport/ -I/usr/include/x86_64-linux-gnu -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/linux/ -shared -fPIC -o liblistSerialsj.so
cp liblistSerialsj.so distrib/linux64/

mkdir -p distrib/linux32
cd libserialport
./autogen.sh
CFLAGS=-m32 ./configure
make clean
make
cd ..
gcc -m32 main.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -Ilibserialport/ -o listSerialC
cp listSerialC distrib/linux32/listSerialC
gcc -m32 jnilib.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -Ilibserialport/ -I/usr/include/x86_64-linux-gnu -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/linux/ -shared -fPIC -o liblistSerialsj.so
cp liblistSerialsj.so distrib/linux32

mkdir -p distrib/arm
cd libserialport
./autogen.sh
./configure --host=arm-linux-gnueabihf
make clean
make
cd ..
arm-linux-gnueabihf-gcc main.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -I/usr/include/x86_64-linux-gnu -Ilibserialport/  -o listSerialC
cp listSerialC distrib/arm/listSerialC
arm-linux-gnueabihf-gcc jnilib.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -I/usr/include/x86_64-linux-gnu -Ilibserialport/ -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/linux/ -shared -fPIC -o liblistSerialsj.so
cp liblistSerialsj.so distrib/arm/

mkdir -p distrib/aarch64
cd libserialport
./autogen.sh
./configure --host=aarch64-linux-gnu
make clean
make
cd ..
aarch64-linux-gnu-gcc main.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -I/usr/include/x86_64-linux-gnu -Ilibserialport/  -o listSerialC
cp listSerialC distrib/aarch64/listSerialC
aarch64-linux-gnu-gcc jnilib.c libserialport/linux_termios.c libserialport/linux.c libserialport/serialport.c -I/usr/include/x86_64-linux-gnu -Ilibserialport/ -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/linux/ -shared -fPIC -o liblistSerialsj.so
cp liblistSerialsj.so distrib/aarch64/
