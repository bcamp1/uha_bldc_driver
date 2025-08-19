#!/bin/bash
./d && sleep 0.3

if [ "$1" = "-b" ]; then
    cmake -S $UHA_BLDC -B $UHA_BLDC/build
fi

cd $UHA_BLDC/build
make flash || exit $?
cd $UHA_BLDC

if [ "$1" = "-u" ]; then
    screen /dev/ttyUSB0 115200
fi

if [ "$1" = "-e" ]; then
    ./e
fi

