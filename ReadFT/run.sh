#!/bin/sh

docker run --device=/dev/ttyUSB0 --platform linux/amd64 --rm -v "$(pwd)":/root/catkin_ws/RoMeLa-Testbed --network host -w /root/catkin_ws/RoMeLa-Testbed -i -t test:latest

