# $NetBSD$
#
# Install the AIC8800D80 firmware into the base set, mirroring
# external/realtek/rtw89.  The dist/ import alone never reaches DESTDIR --
# the sets lists (distrib/sets/lists/base/mi) declare these files, so
# checkflist fails on any full `build.sh distribution` without this
# Makefile (rtw89 lesson, 2026-09-17).
#
# Files follow the vendor loader's non-IPC variant (the combination the
# Linux ground truth ran on the board); see the aic8800 port plan for the
# download order and RAM addresses.

NOMAN=	# define

FILES=	dist/fmacfw_8800d80_u02.bin
FILES+=	dist/fw_adid_8800d80_u02.bin
FILES+=	dist/fw_patch_8800d80_u02.bin
FILES+=	dist/fw_patch_table_8800d80_u02.bin
FILES+=	dist/lmacfw_rf_8800d80_u02.bin
FILES+=	dist/aic_userconfig_8800d80.txt
FILES+=	dist/aic_powerlimit_8800d80.txt
FILES+=	dist/calibmode_8800d80.bin

FILESDIR=	${FIRMWAREDIR}/aic8800u

.include <bsd.files.mk>
