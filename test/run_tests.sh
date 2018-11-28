#!/bin/bash

TARGET_BOARD="/dev/ttyACM0"
INO_FILE="../Marlin/Marlin.ino"

if [ "$1" = "-f" ]; then
    if [ ! -e $TARGET_BOARD ]; then
        echo "$TARGET_BOARD not found."
        exit 1
    fi
    bash flash_mega.sh $TARGET_BOARD $INO_FILE
    sleep 2
fi

if [ -z `pgrep Main` ]; then
  logic > /dev/null 2>&1 &
  LOGIC_TO_BE_STOPPED=1
fi;

cd tests
source env/bin/activate
mkdir -p tmp/

nosetests -v

if [ ! -z "$LOGIC_TO_BE_STOPPED" ]; then
  killall Main
fi;
