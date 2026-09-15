/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by the RK3568 lab (NetBSD/RTL8851BU bring-up).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Chip/net80211 glue between if_rtw89.c, the compat shadow mac80211 and
 * the imported rtw89 core.  M0 skeleton: satisfies the link contract of
 * rtw89_compat.c (shadow mac80211 single-instance helpers); the real
 * core_init/core_start sequencing lands with M1.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include "rtw89_glue.h"

/*
 * The shadow mac80211 has no station-add callback: the per-station chip
 * state (struct rtw89_sta) must be bound before any iterator runs
 * (rtw88 fix d5062cf55632).
 */
void
rtw89_sta_init(struct ieee80211_sta *sta, struct ieee80211_vif *vif,
    struct rtw_dev *rtwdev)
{
	(void)sta;
	(void)vif;
	(void)rtwdev;
}
