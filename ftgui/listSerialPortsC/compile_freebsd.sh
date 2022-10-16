#!/bin/sh -xe

# REQUIRES: devel/openjdk8 
JAVA_INCLUDE_PATH=/usr/local/openjdk8/include

# Only compile for native architecture -- no cross-compiling here
mkdir -p distrib/freebsd
cd libserialport
./autogen.sh
CC=clang ./configure
make clean
make
cd ..
clang main.c libserialport/freebsd.c libserialport/serialport.c -Ilibserialport/ -lusb -o listSerialC
cp listSerialC distrib/freebsd/listSerialC
clang jnilib.c libserialport/freebsd.c libserialport/serialport.c -Ilibserialport/ -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/freebsd/ -shared -fPIC -o liblistSerialsj.so
cp liblistSerialsj.so distrib/freebsd/
