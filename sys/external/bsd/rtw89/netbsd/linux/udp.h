#ifndef _RTW89_LINUX_UDP_H_
#define _RTW89_LINUX_UDP_H_
#include "rtw89_compat.h"

struct udphdr {
	__be16		source;
	__be16		dest;
	__be16		len;
	__be16		check;
} __packed;

static __always_inline __unused struct udphdr *
udp_hdr(const struct sk_buff *skb)
{
	struct iphdr *ip = ip_hdr(skb);

	return (struct udphdr *)((u8 *)ip + ip->ihl * 4);
}

#endif
