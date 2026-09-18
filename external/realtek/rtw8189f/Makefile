# $NetBSD$
#
# Install the rtw8189f (RTL8189FTV / RTL8188F SDIO) firmware into the base
# set, mirroring external/realtek/rtw88 and rtw89.  The dist/ import alone
# never reaches DESTDIR without this Makefile -- the sets lists
# (distrib/sets/lists/base/mi) declare the files, so checkflist fails on
# any full `build.sh distribution` otherwise (rtw89 lesson, 2026-09-17).

NOMAN=	# define

FILES=	dist/Realtek-Firmware-License.txt
FILES+=	dist/rtw8189f_fw.bin

FILESDIR=	${FIRMWAREDIR}/if_rtw8189f

.include <bsd.files.mk>
