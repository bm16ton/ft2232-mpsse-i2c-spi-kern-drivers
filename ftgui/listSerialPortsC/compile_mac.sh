#!/bin/bash -xe

JAVA_INCLUDE_PATH=/opt/jvm/jdk1.8.0/include/

mkdir -p distrib/osx
cd libserialport
./autogen.sh
CC=o64-clang ./configure --host=x86_64-apple-darwin13
make clean
make
cd ..
o64-clang main.c libserialport/serialport.c libserialport/macosx.c -Ilibserialport -framework IOKit -framework CoreFoundation -o listSerialC
cp listSerialC distrib/osx/
o64-clang  jnilib.c libserialport/macosx.c libserialport/serialport.c -framework IOKit -framework CoreFoundation -Ilibserialport/ -I$JAVA_INCLUDE_PATH -I$JAVA_INCLUDE_PATH/linux/ -shared -o liblistSerialsj.dylib
cp liblistSerialsj.dylib distrib/osx/
