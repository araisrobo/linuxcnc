# for ubuntu 8.04: ./configure --enable-simulator --enable-run-in-place

# # Release version, for ubuntu 9.10 (also modify -O2 in Makefile):
# ./configure --prefix=/opt/emc2 --enable-simulator \
#             --enable-build-documentation=pdf \
#             --with-tclConfig=/usr/lib/tcl8.5/tclConfig.sh \
#             --with-tkConfig=/usr/lib/tk8.5/tkConfig.sh 

# Debug Version:
./configure --enable-run-in-place --enable-simulator \
            --enable-build-documentation=pdf \
            --with-tclConfig=/usr/lib/tcl8.5/tclConfig.sh \
            --with-tkConfig=/usr/lib/tk8.5/tkConfig.sh
