#ifndef _RTW89_LINUX_IP_H_
#define _RTW89_LINUX_IP_H_
#include "rtw89_compat.h"

#define	IPPROTO_ICMP	1
#define	IPPROTO_UDP	17
#define	IPPROTO_TCP	6

struct iphdr {
	u8		ihl:4, version:4;
	u8		tos;
	__be16		tot_len;
	__be16		id;
	__be16		frag_off;
	u8		ttl;
	u8		protocol;
	__be16		check;
	__be32		saddr;
	__be32		daddr;
} __packed;

static __always_inline __unused struct iphdr *
ip_hdr(const struct sk_buff *skb)
{
	return (struct iphdr *)skb->data;
}

#endif
