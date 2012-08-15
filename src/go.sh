#!/bin/bash

#  : ${1?"Usage: $0 [wx|wox]"}
#  Script exits here if command-line parameter absent,

case "$1" in
    wx)
	#sim with X:
	./autogen.sh
	./configure --enable-simulator \
		    --enable-build-documentation=no
	CONCURRENCY_LEVEL=`getconf _NPROCESSORS_ONLN`
	echo "make -j${CONCURRENCY_LEVEL}"
    ;;
    wox)
	#sim without X:
	./autogen.sh
	./configure --enable-simulator \
		    --enable-build-documentation=no \
		    --disable-gtk --without-x
	CONCURRENCY_LEVEL=`getconf _NPROCESSORS_ONLN`
	echo "make -j${CONCURRENCY_LEVEL}"
    ;;
    *)
	echo -e "Usage:\t$0 [wx|wox]"
	echo -e "\twx:\tsim with X"
	echo -e "\twox:\tsim without X"
    ;;
esac


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
