#!/bin/bash

if [ "$CROSS_COMPILE" == "NOT_AVAILABLE" ]; then
	echo "Missing CROSS_COMPILE environment variable!"
	exit 1
fi

PLAT='renesas_rcarv4h'

echo "------------------------------------"
echo " -----> Building Basic FW"
echo "------------------------------------"

cd basic-fw-main

scons plat=$PLAT -Q -c
scons plat=$PLAT -Q

cd ..
