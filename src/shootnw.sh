#!/bin/bash
for i in {1..30}
do
echo "s" > /dev/ttyACM0
sleep 5
echo done...
done
