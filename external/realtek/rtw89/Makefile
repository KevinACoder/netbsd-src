# $NetBSD$
#
# Install the rtw89 (RTL8851BU family) firmware into the base set, mirroring
# external/realtek/rtw88.  The dist/ import alone never reached DESTDIR --
# the sets lists (distrib/sets/lists/base/mi) declare these files, so
# checkflist failed on any full `build.sh distribution` until this Makefile
# existed (2026-09-17, found while building the NFS-root lane).

NOMAN=	# define

FILES=	dist/Realtek-Firmware-License.txt
FILES+=	dist/rtw8851b_fw.bin
FILES+=	dist/rtw8851b_fw-1.bin

FILESDIR=	${FIRMWAREDIR}/if_rtw89

.include <bsd.files.mk>
