# for ubuntu 8.04: ./configure --enable-simulator --enable-run-in-place

# # Release version, for ubuntu 9.10 (also modify -O2 in Makefile):
# ./configure --prefix=/opt/emc2 --enable-simulator \
#             --with-tclConfig=/usr/lib/tcl8.5/tclConfig.sh \
#             --with-tkConfig=/usr/lib/tk8.5/tkConfig.sh \
#             # --enable-build-documentation=pdf

# Debug Version:
./configure --enable-run-in-place --enable-simulator \
            --with-tclConfig=/usr/lib/tcl8.5/tclConfig.sh \
            --with-tkConfig=/usr/lib/tk8.5/tkConfig.sh \
            # --enable-build-documentation=pdf \
