#!/bin/bash

./autogen.sh
# run-in-place:
./configure --enable-simulator \
            --enable-build-documentation=no \
	    --disable-gtk --without-x

# install to /opt/emc2:
# ./configure --prefix=/opt/emc2 --enable-simulator \
#             --enable-build-documentation=no

CONCURRENCY_LEVEL=`getconf _NPROCESSORS_ONLN`
echo "make -j${CONCURRENCY_LEVEL}"

# for ARM cross build: 
# sb2 -eR apt-get build-deps emc2-sim
# sb2 -eR ./go.sh
# sb2 -eR make -j4
# # login to ARM terminal:
# dpkg --purge emc2-sim emc2-sim-dev
# cd proj/emc2-dev-arm/src
# make install
# # env-variables for ARM:
# export PATH=$PATH:/opt/emc2/bin
# export MANPATH=$MANPATH:/opt/emc2/sahre/man
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/emc2/lib
