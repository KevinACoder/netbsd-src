/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
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
 * ACPI SAR/DSM link stubs: these are x86 firmware interfaces (acpi.c in
 * the upstream tree, not imported).  On ARM nothing evaluates, and the
 * callers fall back to efuse/regdb data.  "../dist/acpi.h" is spelled out
 * because a bare "acpi.h" collides with the ACPICA include directory.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include "../dist/acpi.h"

int
rtw89_acpi_evaluate_dsm(struct rtw89_dev *rtwdev,
    enum rtw89_acpi_dsm_func func, struct rtw89_acpi_dsm_result *res)
{
	(void)rtwdev;
	(void)func;
	(void)res;
	return -EOPNOTSUPP;
}

int
rtw89_acpi_evaluate_rtag(struct rtw89_dev *rtwdev,
    struct rtw89_acpi_rtag_result *res)
{
	(void)rtwdev;
	(void)res;
	return -EOPNOTSUPP;
}

int
rtw89_acpi_evaluate_sar(struct rtw89_dev *rtwdev,
    struct rtw89_sar_cfg_acpi *cfg)
{
	(void)rtwdev;
	(void)cfg;
	return -EOPNOTSUPP;
}

int
rtw89_acpi_evaluate_dynamic_sar_indicator(struct rtw89_dev *rtwdev,
    struct rtw89_sar_cfg_acpi *cfg, bool *changed)
{
	(void)rtwdev;
	(void)cfg;
	(void)changed;
	return -EOPNOTSUPP;
}

enum rtw89_acpi_sar_subband
rtw89_acpi_sar_get_subband(struct rtw89_dev *rtwdev, u32 center_freq)
{
	(void)rtwdev;
	(void)center_freq;
	return RTW89_ACPI_SAR_2GHZ_SUBBAND;
}

enum rtw89_band
rtw89_acpi_sar_subband_to_band(struct rtw89_dev *rtwdev,
    enum rtw89_acpi_sar_subband subband)
{
	(void)rtwdev;
	(void)subband;
	return RTW89_BAND_2G;
}
