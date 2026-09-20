# $NetBSD$
#
# Mirror external/realtek/Makefile.inc: the firmware directories here
# are files-only (bsd.files.mk) and must not break the `includes` phase
# of a full build (found 2026-09-20, aic8800 NFS-root lane).

FILESOWN=	${FIRMWAREOWN}
FILESGRP=	${FIRMWAREGRP}
FILESMODE=	${FIRMWAREMODE}

includes:
clean:
cleandir:
