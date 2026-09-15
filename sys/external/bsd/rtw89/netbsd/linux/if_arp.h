#ifndef _RTW89_LINUX_IF_ARP_H_
#define _RTW89_LINUX_IF_ARP_H_
#include "rtw89_compat.h"

#define	ARPHRD_ETHER	1

struct arphdr {
	__be16		ar_hrd;
	__be16		ar_pro;
	u8		ar_hln;
	u8		ar_pln;
	__be16		ar_op;
} __packed;

#define	ARPOP_REQUEST	1
#define	ARPOP_REPLY	2

#endif
