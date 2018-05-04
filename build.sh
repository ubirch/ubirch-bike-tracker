#!/bin/sh

binfileName=BikeTracker.bin
buildtarget=build
binfile=./$buildtarget/$binfileName

if [ ! -d $buildtarget ]; then
  mkdir $buildtarget
fi

if [ -f $binfile ]; then
  rm $binfile
fi

particle compile electron --target 0.7.0 ./src/BikeTracker.ino project.properties ./ --saveTo $binfile
#particle flash --usb $binfile
