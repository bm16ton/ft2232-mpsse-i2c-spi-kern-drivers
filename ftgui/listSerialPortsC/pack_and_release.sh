#!/bin/bash -xe

VERSION=$1

if [ x$VERSION = "x" ]
then
echo "Please provide a version number"
exit 1
fi

./compile_win.sh
./compile_linux.sh
./compile_mac.sh

mv distrib liblistSerials-$VERSION
zip -r liblistSerials-$VERSION.zip  liblistSerials-$VERSION

echo scp liblistSerials-$VERSION.zip root@downloads-02.arduino.cc:/var/www/files/liblistSerials/
shasum liblistSerials-$VERSION.zip

rm -rf liblistSerials-$VERSION
