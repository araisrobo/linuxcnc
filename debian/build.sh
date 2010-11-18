#!/bin/bash

#usage: source ./build.sh
RELEASE=2.4.0-nurbs-2010.10.18
./configure sim
nice debuild -S
sudo nice pbuilder build ../../emc2_${RELEASE}.dsc
cp -v \
  /var/cache/pbuilder/result/emc2-sim_${RELEASE}_i386.deb \
  ~/tmp
