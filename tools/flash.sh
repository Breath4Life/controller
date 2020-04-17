#!/bin/bash



if [ ! -z $1 ] 
then 
  PORT=$1
else
  PORT=/dev/ttyACM0
fi

echo "Please press the reset button. Waiting on the USB device ..."

while [ ! -e $PORT ];
do
  sleep 0.1
done
echo "Bootloader ready. Flashing ..."
avrdude -C $(dirname $0)/avrdude.conf -v -v -patmega2560 -cwiring -P $PORT -b115200 -D -U flash:w:main.hex:i
