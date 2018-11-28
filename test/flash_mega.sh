#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "usage : ./flash.sh board file.ino"
    exit -1
fi

TARGET_BOARD=$1
INO_FILE=$2
arduino --upload --port $TARGET_BOARD --board arduino:avr:mega:cpu=atmega2560 $INO_FILE
